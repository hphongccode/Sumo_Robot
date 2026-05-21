#include "robot_control.h"
#include "main.h"
#include "Motor.h"
#include "line_sensor.h"
#include "ultrasonic.h"
#include "fuzzy.h"
#include "message.h"

extern volatile uint8_t line_flag;
extern volatile uint8_t line_dir;

typedef enum {
    STATE_NORMAL = 0,
    STATE_ESCAPE_REVERSE,
    STATE_ESCAPE_TURN,
    STATE_SEARCH,
    STATE_IDLE
} RobotState;

static RobotState current_state    = STATE_NORMAL;
static uint8_t    escape_mask      = 0;
static uint32_t   state_start_time = 0;
static uint32_t   last_seen_time   = 0;

#define TIME_REVERSE       450
#define TIME_TURN          500

#define OPPONENT_DIST_CM   40u
#define SEARCH_TIMEOUT_MS  2500u
#define SEARCH_DURATION_MS 3000u
#define IDLE_DURATION_MS   5000u

/* ══════════════════════════════════════════════════════════
   [SỬA BUG 2] CHỐNG GLITCH – THAY THẾ BỘ LỌC IIR
   ──────────────────────────────────────────────────────────
   Bộ lọc IIR cũ (ULTRA_ALPHA = 0.45) gây 2 vấn đề lớn:
   
   ① Sau escape, filter reset về 999cm → robot "mù" ~300ms
      (cần 8+ lần đọc để hội tụ) → xoay tròn vô ích.
   
   ② Phản hồi chậm: vật cản ở 25cm nhưng filter hiển thị
      300+cm → fuzzy nghĩ "FAR" → xoay NGƯỢC hướng vật cản!
   
   GIẢI PHÁP MỚI: Ultra_Safe() – chỉ thay giá trị lỗi
   (-1 timeout từ HC-SR04) bằng lần đọc hợp lệ gần nhất.
   → Phản hồi TỨC THỜI, không có độ trễ hội tụ.
   → Không cần reset khi chuyển state.
   ══════════════════════════════════════════════════════════ */
static float last_ok_l = 999.0f;  /* Giá trị hợp lệ gần nhất – TRÁI  */
static float last_ok_m = 999.0f;  /* Giá trị hợp lệ gần nhất – GIỮA  */
static float last_ok_r = 999.0f;  /* Giá trị hợp lệ gần nhất – PHẢI  */

static UltraState Ultra_Safe(void)
{
    UltraState raw = Ultra_ReadAll();
    UltraState u;

    /* Đọc hợp lệ (> 0.5cm): dùng luôn + ghi nhớ.
       Đọc lỗi (≤ 0, timeout): giữ giá trị cũ, không nhảy về 0 hay 999. */
    u.left  = (raw.left  > 0.5f) ? (last_ok_l = raw.left)  : last_ok_l;
    u.mid   = (raw.mid   > 0.5f) ? (last_ok_m = raw.mid)   : last_ok_m;
    u.right = (raw.right > 0.5f) ? (last_ok_r = raw.right) : last_ok_r;

    return u;
}

/* ══════════════════════════════════════════════════════════
   KHÓA HƯỚNG khi phát hiện vật cản CHỈ ở 1 bên
   ──────────────────────────────────────────────────────────
   Khi cảm biến BÊN phát hiện gần nhưng GIỮA chưa thấy,
   giữ motor output ổn định trong LOCK_CHARGE_MS ms.
   → Tránh giật do sonar bên nhiễu cơ học (rung lắc).
   ══════════════════════════════════════════════════════════ */
#define LOCK_CHARGE_MS     100u

static uint8_t  charge_locked    = 0;
static uint32_t charge_lock_time = 0;
static int16_t  locked_ls = 0, locked_rs = 0;

/* ═══════════════════════════════════════════════════════ */
static void Drive(int16_t ls, int16_t rs)
{
    Motor_Set(ls, rs);
    MSG_SendRobotState(ls, rs, (uint8_t)current_state);
}

static uint8_t opponent_detected(const UltraState *u)
{
    return (u->left  < OPPONENT_DIST_CM ||
            u->mid   < OPPONENT_DIST_CM ||
            u->right < OPPONENT_DIST_CM) ? 1u : 0u;
}

/* ═══════════════════════════════════════════════════════ */
/*  Escape helpers (GIỮ NGUYÊN)                            */
/* ═══════════════════════════════════════════════════════ */
static void escape_get_motors(uint8_t mask, int16_t *ls, int16_t *rs)
{
    uint8_t f = mask & LINE_FRONT, b = mask & LINE_BACK;
    uint8_t l = mask & LINE_LEFT,  r = mask & LINE_RIGHT;

    if(f && l) { *ls = -90; *rs = -70; return; }
    if(f && r) { *ls = -70; *rs = -90; return; }
    if(b && l) { *ls = 100; *rs =  60; return; }
    if(b && r) { *ls =  60; *rs = 100; return; }

    if(f) { *ls = -90; *rs = -90; return; }
    if(b) { *ls =  90; *rs =  90; return; }
    if(l) { *ls = -90; *rs = -60; return; }
    if(r) { *ls = -60; *rs = -90; return; }
    *ls = 0; *rs = 0;
}

static void escape_get_turn(uint8_t mask, int16_t *ls, int16_t *rs)
{
    if(mask & LINE_RIGHT) { *ls = -90; *rs =  90; }
    else                  { *ls =  90; *rs = -90; }
}

/* ═══════════════════════════════════════════════════════ */
/*  Reset                                                   */
/* ═══════════════════════════════════════════════════════ */
void Robot_ResetState(void)
{
    current_state    = STATE_NORMAL;
    escape_mask      = 0;
    state_start_time = 0;
    last_seen_time   = HAL_GetTick();
    charge_locked    = 0;

    __disable_irq();
    line_flag = 0;
    line_dir  = 0;
    __enable_irq();
}

/* ═══════════════════════════════════════════════════════ */
/*  Main loop                                               */
/* ═══════════════════════════════════════════════════════ */
void Robot_Run(void)
{
    uint32_t now = HAL_GetTick();

    /* ══════════════════════════════════════════════════════
       [SỬA BUG 1] Xử lý ngắt cảm biến mép
       ──────────────────────────────────────────────────────
       BUG CŨ: Khi đang escape mà ngắt line fire liên tục
       (robot còn đè vạch), code cũ gọi `return` ngay lập tức
       → switch(current_state) KHÔNG BAO GIỜ CHẠY
       → timer (now - state_start_time) không được kiểm tra
       → robot KẸT trong escape, không chuyển sang TURN/NORMAL
       → biểu hiện: lùi 10cm rồi xoay tại chỗ 3-4 vòng.

       SỬA: Khi đang escape mà có ngắt mới, CHỈ gộp hướng
       (escape_mask |= new_mask), KHÔNG return. State machine
       vẫn chạy bình thường → timer luôn được kiểm tra →
       robot chuyển state đúng thời điểm.
       ══════════════════════════════════════════════════════ */
    if (line_flag)
    {
        __disable_irq();
        uint8_t new_mask = line_dir;
        line_dir  = 0;
        line_flag = 0;
        __enable_irq();

        if (current_state == STATE_ESCAPE_REVERSE ||
            current_state == STATE_ESCAPE_TURN)
        {
            /* [SỬA] Gộp hướng mới, KHÔNG return
               → fall through xuống state machine bên dưới */
            escape_mask |= new_mask;
        }
        else
        {
            /* Lần đầu chạm line → khởi động escape */
            escape_mask      = new_mask;
            current_state    = STATE_ESCAPE_REVERSE;
            state_start_time = now;

            int16_t ls, rs;
            escape_get_motors(escape_mask, &ls, &rs);
            Drive(ls, rs);
            return;
        }
    }

    /* ── 2. State machine ── */
    switch (current_state)
    {
        /* ─────────────────────────────────────────── */
        case STATE_ESCAPE_REVERSE:
        {
            if ((now - state_start_time) < TIME_REVERSE) {
                int16_t ls, rs;
                escape_get_motors(escape_mask, &ls, &rs);
                Drive(ls, rs);
            } else {
                if (escape_mask & LINE_FRONT) {
                    current_state    = STATE_ESCAPE_TURN;
                    state_start_time = now;
                } else {
                    current_state  = STATE_NORMAL;
                    last_seen_time = now;
                    charge_locked  = 0;
                }
            }
            break;
        }

        /* ─────────────────────────────────────────── */
        case STATE_ESCAPE_TURN:
        {
            if ((now - state_start_time) < TIME_TURN) {
                int16_t ls, rs;
                escape_get_turn(escape_mask, &ls, &rs);
                Drive(ls, rs);
            } else {
                LineState s = Line_Read();
                if (!s.front && !s.back && !s.left && !s.right) {
                    current_state  = STATE_NORMAL;
                    last_seen_time = now;
                    charge_locked  = 0;
                } else {
                    current_state    = STATE_ESCAPE_REVERSE;
                    state_start_time = now;
                }
            }
            break;
        }

        /* ─────────────────────────────────────────── */
        case STATE_NORMAL:
        default:
        {
            /* [SỬA BUG 2] Dùng Ultra_Safe() – phản hồi tức thời,
               không bị trễ 300ms như bộ lọc IIR cũ */
            UltraState u = Ultra_Safe();
            int16_t ls, rs;

            if (opponent_detected(&u)) {
                last_seen_time = now;

                /* Khóa hướng: giữ motor ổn định khi chỉ bên phát hiện */
                if (charge_locked && (now - charge_lock_time) < LOCK_CHARGE_MS) {
                    Drive(locked_ls, locked_rs);
                } else {
                    Fuzzy_Control(u.left, u.mid, u.right, &ls, &rs);
                    int16_t out_ls = (int16_t)(ls * 0.9f);
                    int16_t out_rs = (int16_t)(rs * 0.9f);

                    /* Chỉ cảm biến BÊN thấy (giữa chưa) → khóa hướng */
                    uint8_t side_only = (u.mid >= OPPONENT_DIST_CM) &&
                                        (u.left < OPPONENT_DIST_CM ||
                                         u.right < OPPONENT_DIST_CM);
                    if (side_only) {
                        charge_locked    = 1;
                        charge_lock_time = now;
                        locked_ls = out_ls;
                        locked_rs = out_rs;
                    } else {
                        charge_locked = 0;
                    }

                    Drive(out_ls, out_rs);
                }
            } else {
                charge_locked = 0;
                if ((now - last_seen_time) >= SEARCH_TIMEOUT_MS) {
                    current_state    = STATE_SEARCH;
                    state_start_time = now;
                    Drive(0, 0);
                } else {
                    Fuzzy_Control(u.left, u.mid, u.right, &ls, &rs);
                    Drive(ls * 0.9f, rs * 0.9f);
                }
            }
            break;
        }

        /* ─────────────────────────────────────────── */
        case STATE_SEARCH:
        {
            UltraState u = Ultra_Safe();

            if (opponent_detected(&u)) {
                last_seen_time = now;
                current_state  = STATE_NORMAL;
                int16_t ls, rs;
                Fuzzy_Control(u.left, u.mid, u.right, &ls, &rs);
                Drive(ls * 0.9f, rs * 0.9f);
                break;
            }

            if ((now - state_start_time) < SEARCH_DURATION_MS) {
                Drive(80, -80);   /* Xoay PHẢI – ưu tiên phải */
            } else {
                current_state    = STATE_IDLE;
                state_start_time = now;
                Drive(0, 0);
            }
            break;
        }

        /* ─────────────────────────────────────────── */
        case STATE_IDLE:
        {
            UltraState u = Ultra_Safe();

            if (opponent_detected(&u)) {
                last_seen_time = now;
                current_state  = STATE_NORMAL;
                int16_t ls, rs;
                Fuzzy_Control(u.left, u.mid, u.right, &ls, &rs);
                Drive(ls * 0.9f, rs * 0.9f);
                break;
            }

            Drive(0, 0);

            if ((now - state_start_time) >= IDLE_DURATION_MS) {
                current_state    = STATE_SEARCH;
                state_start_time = now;
                last_seen_time   = now;
            }
            break;
        }
    }
}
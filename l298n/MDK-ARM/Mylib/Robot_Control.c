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

#define TIME_REVERSE       300
#define TIME_TURN          600

#define OPPONENT_DIST_CM   40u
#define SEARCH_TIMEOUT_MS  2500u
#define SEARCH_DURATION_MS 2500u
#define IDLE_DURATION_MS   5000u

/* ═══════════════════════════════════════════════════════ */
/*  Wrapper: set motor + gửi data về ESP cùng lúc          */
/*  Dùng hàm này thay cho Motor_Set trực tiếp              */
/* ═══════════════════════════════════════════════════════ */
static void Drive(int16_t ls, int16_t rs)
{
    Motor_Set(ls, rs);
    MSG_SendRobotState(ls, rs, (uint8_t)current_state);
}

/* ═══════════════════════════════════════════════════════ */
/*  Helper: phát hiện đối thủ                              */
/* ═══════════════════════════════════════════════════════ */
static uint8_t opponent_detected(const UltraState *u)
{
    return (u->left  < OPPONENT_DIST_CM ||
            u->mid   < OPPONENT_DIST_CM ||
            u->right < OPPONENT_DIST_CM) ? 1u : 0u;
}

/* ═══════════════════════════════════════════════════════ */
/*  Escape helpers                                          */
/* ═══════════════════════════════════════════════════════ */
static void escape_get_motors(uint8_t mask, int16_t *ls, int16_t *rs)
{
    uint8_t f = mask & LINE_FRONT, b = mask & LINE_BACK;
    uint8_t l = mask & LINE_LEFT,  r = mask & LINE_RIGHT;

    if(f && l) { *ls = -90; *rs = -70; return; }
    if(f && r) { *ls = -70; *rs = -90; return; }
    if(b && l) { *ls =  100; *rs =  60; return; }
    if(b && r) { *ls =  60;  *rs = 100; return; }

    if(f) { *ls = -90; *rs = -90; return; }
    if(b) { *ls =  90; *rs =  90; return; }
    if(l) { *ls = -90; *rs = -50; return; }
    if(r) { *ls = -50; *rs = -90; return; }
    *ls = 0; *rs = 0;
}

static void escape_get_turn(uint8_t mask, int16_t *ls, int16_t *rs)
{
    if(mask & LINE_RIGHT) { *ls = -60; *rs =  80; }
    else                  { *ls =  80; *rs = -60; }
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

    /* ── 1. Ngắt cảm biến mép ── */
    if (line_flag)
    {
        __disable_irq();
        escape_mask = line_dir;
        line_dir    = 0;
        line_flag   = 0;
        __enable_irq();

        current_state    = STATE_ESCAPE_REVERSE;
        state_start_time = now;

        int16_t ls, rs;
        escape_get_motors(escape_mask, &ls, &rs);
        Drive(ls, rs);   // ← thay Motor_Set
        return;
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
            UltraState u = Ultra_ReadAll();
            int16_t ls, rs;

            if (opponent_detected(&u)) {
                last_seen_time = now;
                Fuzzy_Control(u.left, u.mid, u.right, &ls, &rs);
                Drive(ls * 0.9f, rs * 0.9f);
            } else {
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
            UltraState u = Ultra_ReadAll();

            if (opponent_detected(&u)) {
                last_seen_time = now;
                current_state  = STATE_NORMAL;
                int16_t ls, rs;
                Fuzzy_Control(u.left, u.mid, u.right, &ls, &rs);
                Drive(ls * 0.9f, rs * 0.9f);
                break;
            }

            if ((now - state_start_time) < SEARCH_DURATION_MS) {
                Drive(80, -80);
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
            UltraState u = Ultra_ReadAll();

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
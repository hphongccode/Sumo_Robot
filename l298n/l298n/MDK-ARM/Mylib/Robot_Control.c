#include "robot_control.h"
#include "main.h"
#include "Motor.h"
#include "line_sensor.h"
#include "ultrasonic.h"
#include "fuzzy.h"

extern volatile uint8_t line_flag;
extern volatile uint8_t line_dir;

typedef enum { STATE_NORMAL = 0, STATE_ESCAPE_REVERSE, STATE_ESCAPE_TURN } RobotState;

static RobotState current_state    = STATE_NORMAL;
static uint8_t    escape_mask      = 0;
static uint32_t   state_start_time = 0;

#define TIME_REVERSE  400
#define TIME_TURN     550


/* Tính motor output t? bitmask, x? lý d? 16 t? h?p */
static void escape_get_motors(uint8_t mask, int16_t *ls, int16_t *rs)
{
    uint8_t f = mask & LINE_FRONT, b = mask & LINE_BACK;
    uint8_t l = mask & LINE_LEFT,  r = mask & LINE_RIGHT;
		/*
		if(f && b && l) { *ls = -80; *rs =  80; return; }  // B? ép 3 phía tr? ph?i ? xoay ph?i thoát
    if(f && b && r) { *ls =  80; *rs = -80; return; }  // B? ép 3 phía tr? trái ? xoay trái thoát
    if(f && l && r) { *ls = -90; *rs = -90; return; }  // Phía tru?c + 2 bên ? lùi th?ng
    if(b && l && r) { *ls =  90; *rs =  90; return; }  // Phía sau + 2 bên ? ti?n th?ng
		*/
    if(f && l) { *ls = -90; *rs = -70; return; }  // Lùi l?ch ph?i
    if(f && r) { *ls = -70; *rs = -90;  return; }  // Lùi l?ch trái
    if(b && l) { *ls =  100; *rs =  60; return; }  // Ti?n l?ch ph?i
    if(b && r) { *ls =  60; *rs =  100;  return; }  // Ti?n l?ch trái
   
		if(f)      { *ls = -100;  *rs = -100;  return; }
    if(b)      { *ls =  100;  *rs =  100;  return; }
    if(l)      { *ls =  -90;  *rs = -70;  return; }  // Xoay ph?i
    if(r)      { *ls = -70;  *rs =  -90;  return; }  // Xoay trái
    *ls = 0; *rs = 0;
}

/* Hu?ng quay v? gi?a sân sau khi dã lùi d? */
static void escape_get_turn(uint8_t mask, int16_t *ls, int16_t *rs)
{
    if(mask & LINE_RIGHT) { *ls = -80; *rs =  80; }   // Bên ph?i có line ? quay trái
    else                  { *ls =  80; *rs = -80; }   // M?c d?nh ? quay ph?i
}

void Robot_Run(void)
{
    uint32_t now = HAL_GetTick();

    /* ===== 1. X? LÝ C? NG?T ===== */
    if (line_flag)
    {
        /* Atomic read+clear: tránh race condition gi?a ISR và main loop */
        __disable_irq();
        escape_mask = line_dir;
        line_dir    = 0;
        line_flag   = 0;
        __enable_irq();

        current_state    = STATE_ESCAPE_REVERSE;
        state_start_time = now;

        int16_t ls, rs;
        escape_get_motors(escape_mask, &ls, &rs);
        Motor_Set(ls, rs);
        return;
    }

    /* ===== 2. STATE MACHINE ===== */
    switch (current_state)
    {
        case STATE_ESCAPE_REVERSE:
        {
            if ((now - state_start_time) < TIME_REVERSE) {
                int16_t ls, rs;
                escape_get_motors(escape_mask, &ls, &rs);
                Motor_Set(ls, rs);
            } else {
                /* Side-only (left/right) dã thoát b?ng xoay ? v? NORMAL ngay */
                if (escape_mask & (LINE_FRONT )) {
                    current_state    = STATE_ESCAPE_TURN;
                    state_start_time = now;
                } else {
                    current_state = STATE_NORMAL;
                }
            }
            break;
        }

        case STATE_ESCAPE_TURN:
        {
            if ((now - state_start_time) < TIME_TURN) {
                int16_t ls, rs;
                escape_get_turn(escape_mask, &ls, &rs);
                Motor_Set(ls, rs);
            } else {
                LineState s = Line_Read();
                if (!s.front && !s.back && !s.left && !s.right) {
                    current_state = STATE_NORMAL;
                } else {
                    current_state    = STATE_ESCAPE_REVERSE;  // V?n dính line ? lùi l?i
                    state_start_time = now;
                }
            }
            break;
        }

        case STATE_NORMAL:
        default:
        {
            /* Polling fallback – b?t line k? c? khi interrupt miss do t?c d? cao (s? c? 1) */
            /*LineState poll = Line_Read();
             uint8_t poll_mask = 0;
            if(poll.front) poll_mask |= LINE_FRONT;
            if(poll.back)  poll_mask |= LINE_BACK;
            if(poll.left)  poll_mask |= LINE_LEFT;
            if(poll.right) poll_mask |= LINE_RIGHT;

            if(poll_mask) {
                __disable_irq();
                escape_mask = poll_mask | line_dir;
                line_dir    = 0;
                line_flag   = 0;
                __enable_irq();
                current_state    = STATE_ESCAPE_REVERSE;
                state_start_time = now;
                int16_t ls, rs;
                escape_get_motors(escape_mask, &ls, &rs);
                Motor_Set(ls, rs);
                break;
            }
						*/
            UltraState u = Ultra_ReadAll();
            int16_t ls, rs;
            Fuzzy_Control(u.left, u.mid, u.right, &ls, &rs);
            Motor_Set(ls*0.9, rs*0.9);
            break;
        }
    }
}
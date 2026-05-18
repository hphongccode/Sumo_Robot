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

#define TIME_REVERSE  300
#define TIME_TURN     600

void Robot_ResetState(void) {
    current_state    = STATE_NORMAL;
    escape_mask      = 0;
    state_start_time = 0;

    // Clear any pending line sensor flags
    __disable_irq();
    line_flag = 0;
    line_dir  = 0;
    __enable_irq();
}
/* T�nh motor output t? bitmask, x? l� d? 16 t? h?p */
static void escape_get_motors(uint8_t mask, int16_t *ls, int16_t *rs)
{
    uint8_t f = mask & LINE_FRONT, b = mask & LINE_BACK;
    uint8_t l = mask & LINE_LEFT,  r = mask & LINE_RIGHT;
		/*
		if(f && b && l) { *ls = -80; *rs =  80; return; }  // B? �p 3 ph�a tr? ph?i ? xoay ph?i tho�t
    if(f && b && r) { *ls =  80; *rs = -80; return; }  // B? �p 3 ph�a tr? tr�i ? xoay tr�i tho�t
    if(f && l && r) { *ls = -90; *rs = -90; return; }  // Ph�a tru?c + 2 b�n ? l�i th?ng
    if(b && l && r) { *ls =  90; *rs =  90; return; }  // Ph�a sau + 2 b�n ? ti?n th?ng
		*/
    if(f && l) { *ls = -90; *rs = -70; return; }  // L�i l?ch ph?i
    if(f && r) { *ls = -70; *rs = -90;  return; }  // L�i l?ch tr�i
    if(b && l) { *ls =  100; *rs =  60; return; }  // Ti?n l?ch ph?i
    if(b && r) { *ls =  60; *rs =  100;  return; }  // Ti?n l?ch tr�i

		if(f)      { *ls = -100;  *rs = -100;  return; }
    if(b)      { *ls =  100;  *rs =  100;  return; }
    if(l)      { *ls =  -90;  *rs = -70;  return; }  // Xoay ph?i
    if(r)      { *ls = -70;  *rs =  -90;  return; }  // Xoay tr�i
    *ls = 0; *rs = 0;
}

/* Hu?ng quay v? gi?a s�n sau khi d� l�i d? */
static void escape_get_turn(uint8_t mask, int16_t *ls, int16_t *rs)
{
    if(mask & LINE_RIGHT) { *ls = -80; *rs =  80; }   // B�n ph?i c� line ? quay tr�i
    else                  { *ls =  80; *rs = -80; }   // M?c d?nh ? quay ph?i
}

void Robot_Run(void)
{
    uint32_t now = HAL_GetTick();

    /* ===== 1. X? L� C? NG?T ===== */
    if (line_flag)
    {
        /* Atomic read+clear: tr�nh race condition gi?a ISR v� main loop */
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
                /* Side-only (left/right) d� tho�t b?ng xoay ? v? NORMAL ngay */
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
                    current_state    = STATE_ESCAPE_REVERSE;  // V?n d�nh line ? l�i l?i
                    state_start_time = now;
                }
            }
            break;
        }

        case STATE_NORMAL:
        default:
        {
            /* Polling fallback � b?t line k? c? khi interrupt miss do t?c d? cao (s? c? 1) */
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
            Motor_Set(ls*0.8, rs*0.8);
            break;
        }
    }
}

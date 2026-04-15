#include "robot_control.h"
#include "main.h"         // Ð? s? d?ng hàm HAL_GetTick()
#include "Motor.h"
#include "line_sensor.h"
#include "ultrasonic.h"
#include "fuzzy.h"

/* L?y các bi?n c? ng?t t? file line_sensor.c */
extern volatile uint8_t line_flag;
extern volatile uint8_t line_dir;

/* Ð?nh nghia các tr?ng thái c?a Robot */
typedef enum {
    STATE_NORMAL = 0,
    STATE_ESCAPE_REVERSE,
    STATE_ESCAPE_TURN
} RobotState;

/* Các bi?n c?c b? qu?n lý State Machine */
static RobotState current_state = STATE_NORMAL;
static uint8_t current_escape_dir = 0;
static uint32_t state_start_time = 0;

/* Tùy ch?nh thông s? th?i gian (ms) - C?n test th?c t? d? tinh ch?nh */
#define TIME_REVERSE 400 // Th?i gian lùi g?p d? tri?t tiêu quán tính
#define TIME_TURN    300 // Th?i gian xoay robot hu?ng vào trong sân

void Robot_Run(void)
{
    uint32_t now = HAL_GetTick();

    /* ===== 1. UU TIÊN S? 1: NH?N NG?T LINE ===== */
    if (line_flag)
    {
        line_flag = 0; 
        
        // C?p nh?t tr?ng thái sang lùi g?p ngay l?p t?c
        current_state = STATE_ESCAPE_REVERSE;
        current_escape_dir = line_dir;
        state_start_time = now;
        
        // Phanh g?p v?i t?c d? cao nh?t d? ch?ng tru?t qua line
        switch(current_escape_dir) {
            case 1: Motor_Set(-85, -85); break; // V?ch tru?c -> Lùi Max t?c
            case 2: Motor_Set(85, 85);   break; // V?ch sau -> Ti?n Max t?c
            case 3: Motor_Set(85, -85);  break; // V?ch trái -> Xoay ph?i Max t?c
            case 4: Motor_Set(-85, 85);  break; // V?ch ph?i -> Xoay trái Max t?c
            default: Motor_Set(0, 0); break;
        }
        return; // Thoát hàm ngay d? không ch?y Fuzzy
    }

    /* ===== 2. X? LÝ MÁY TR?NG THÁI (STATE MACHINE) ===== */
    switch (current_state)
    {
        case STATE_ESCAPE_REVERSE:
            if ((now - state_start_time) < TIME_REVERSE) 
            {
                // Ðang trong th?i gian lùi g?p -> Duy trì t?c d? phanh
                switch(current_escape_dir) {
                    case 1: Motor_Set(-85, -85); break;
                    case 2: Motor_Set(85, 85);   break;
                    case 3: Motor_Set(85, -85);  break; 
                    case 4: Motor_Set(-85, 85);  break; 
                }
            } 
            else 
            {
                // H?t th?i gian lùi. N?u là v?ch tru?c/sau thì chuy?n sang xoay d?u.
                // N?u là v?ch trái/ph?i thì d?ng tác xoay ? trên dã d? thoát, v? NORMAL.
                if (current_escape_dir == 1 || current_escape_dir == 2) {
                    current_state = STATE_ESCAPE_TURN;
                    state_start_time = now;
                } else {
                    current_state = STATE_NORMAL;
                }
            }
            break;

        case STATE_ESCAPE_TURN:
            if ((now - state_start_time) < TIME_TURN) 
            {
                // Xoay robot hu?ng vào gi?a sân (Ðã tang l?c lên 85 d? th?ng ma sát tinh)
                Motor_Set(85, -85); 
            } 
            else 
            {
                // H?t th?i gian xoay, ki?m tra xem dã thoát v?ch chua
                LineState s = Line_Read();
                if (!s.front && !s.back && !s.left && !s.right) {
                    current_state = STATE_NORMAL; // Ðã an toàn
                } else {
                    // Xui x?o v?n k?t v?ch -> kích ho?t lùi l?i t? d?u
                    current_state = STATE_ESCAPE_REVERSE;
                    state_start_time = now;
                }
            }
            break;

        case STATE_NORMAL:
        default:
        {
            /* ===== 3. CH? Ð? BÌNH THU?NG (FUZZY LOGIC) ===== */
            UltraState u = Ultra_ReadAll();

            int16_t ls, rs;
            Fuzzy_Control(u.left, u.mid, u.right, &ls, &rs);

            // B? gi?m gi?t (x 0.9) d? gi? nguyên s?c m?nh t?n công
            Motor_Set(ls, rs);
        }
        break;
    }
}

#include "message.h"
#include "uart.h" // Sử dụng lại thư viện gửi dữ liệu đã tạo ở bước trước
#include "Robot_Control.h"
#include "Motor.h"

extern float speed_multiplier;
extern uint8_t robot_mode;
/**
 * @brief Hàm phân giải lệnh nhận được từ ESP32
 * @param data: Con trỏ đến bộ đệm chứa dữ liệu
 * @param len: Độ dài dữ liệu thực tế
 */

void CMD_Process(uint8_t *data, uint16_t len) {
    // Chuyển đổi dữ liệu nhận được thành chuỗi (String) để dễ xử lý
    char msg[MAX_CMD_LEN];
    if (len >= MAX_CMD_LEN) len = MAX_CMD_LEN - 1;

    memcpy(msg, data, len);
    msg[len] = '\0'; // Kết thúc chuỗi

    /* BẮT ĐẦU PHÂN GIẢI LỆNH */

    // Kiểm tra lệnh tiến
    if (strstr(msg, "PWR:0") != NULL) {
            speed_multiplier = 0.0f;
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
            // UART_DMA_SendString("STM32: Power OFF, LED PC13 OFF\r\n");
        }
        // BẬT (PWR:1)
        else if (strstr(msg, "PWR:1") != NULL) {
            speed_multiplier = 1.0f;
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
            // UART_DMA_SendString("STM32: Power ON, LED PC13 ON\r\n");
        }
        // CHUYỂN SANG MODE AUTO
        else if (strstr(msg, "MODE:0") != NULL) {
            robot_mode = 0;
            UART_DMA_SendString("STM32: Switched to AUTO Mode\r\n");
        }
        // CHUYỂN SANG MODE MANUAL
        else if (strstr(msg, "MODE:1") != NULL) {
            robot_mode = 1;
            UART_DMA_SendString("STM32: Switched to MANUAL Mode\r\n");
        }

        // ----------------------------------------------------
        // [ĐIỀU KHIỂN THỦ CÔNG] Chỉ thực thi nếu robot_mode == 1
        // ----------------------------------------------------

        // Kiểm tra lệnh tiến (fw)
        else if (robot_mode == 1) {

                /* --- ĐIỀU KHIỂN TIẾN --- */
				if (strstr(msg, "FW:1") != NULL) { Action_MoveForward(); }
				else if (strstr(msg, "BW:1") != NULL) { Action_MoveBackward(); }
				else if (strstr(msg, "RL:1") != NULL) { Action_TurnLeft(); }
				else if (strstr(msg, "RR:1") != NULL) { Action_TurnRight(); }
				// Nếu có bất kỳ lệnh nào chứa ":0" (Nhả nút)
				else if (strstr(msg, ":0") != NULL) {
					Action_Stop();
				}
            }

        // ----------------------------------------------------
        // Trường hợp lệnh không xác định
        // ----------------------------------------------------
        else {
            UART_DMA_SendString("STM32: Unknown Command!\r\n");
        }
}

/* ĐỊNH NGHĨA CÁC HÀNH ĐỘNG CHI TIẾT */
void Action_MoveForward(void) {
	Motor_Set(85, 85);
}

void Action_MoveBackward(void) {
	Motor_Set(-85, -85);
}

void Action_TurnLeft(void) {
    Motor_Set(-85, 85);
}

void Action_TurnRight(void) {
    Motor_Set(85, -85);
}

void Action_Stop(void) {
    Motor_Set(0, 0);
}

#include <stdlib.h>
#include "message.h"
#include "uart.h" // Sử dụng lại thư viện gửi dữ liệu đã tạo ở bước trước
#include "Robot_Control.h"
#include "Motor.h"

extern volatile float speed_multiplier;
static int speed = 85; // speed >= 0 & <= 100
extern volatile uint8_t robot_mode;
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

    uint8_t handled = 0; // Cờ đánh dấu đã xử lý lệnh hợp lệ

    /* BẮT ĐẦU PHÂN GIẢI LỆNH */

    // Kiểm tra lệnh tiến
    if (strstr(msg, "PWR:0") != NULL) {
        speed_multiplier = 0.0f;
        // Bỏ qua PC13 vì mạch mới có thể không dùng
        // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); 
        handled = 1;
    }
    if (strstr(msg, "PWR:1") != NULL) {
        speed_multiplier = 1.0f;
        // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        handled = 1;
    }
    if (strstr(msg, "MODE:0") != NULL) {
        Robot_ResetState();
        robot_mode = 0;
        UART_DMA_SendString("STM32: Switched to AUTO Mode\r\n");
        handled = 1;
    }
    if (strstr(msg, "MODE:1") != NULL) {
        Motor_Set(0, 0);
        robot_mode = 1;
        UART_DMA_SendString("STM32: Switched to MANUAL Mode\r\n");
        handled = 1;
    }
    
    char *speed_ptr = strstr(msg, "SPEED:");
    if (speed_ptr != NULL) {
        speed = atoi(speed_ptr + 6);
        handled = 1;
    }

    // ----------------------------------------------------
    // [ĐIỀU KHIỂN THỦ CÔNG] Chỉ thực thi nếu robot_mode == 1
    // ----------------------------------------------------
    if (robot_mode == 1) {
        if (strstr(msg, "FW:1") != NULL) { Action_MoveForward(); handled = 1; }
        else if (strstr(msg, "BW:1") != NULL) { Action_MoveBackward(); handled = 1; }
        else if (strstr(msg, "RL:1") != NULL) { Action_TurnLeft(); handled = 1; }
        else if (strstr(msg, "RR:1") != NULL) { Action_TurnRight(); handled = 1; }
        // Nhả nút - chỉ khớp đúng lệnh thả nút điều khiển
        else if (strstr(msg, "FW:0") != NULL ||
                 strstr(msg, "BW:0") != NULL ||
                 strstr(msg, "RL:0") != NULL ||
                 strstr(msg, "RR:0") != NULL) {
            Action_Stop();
            handled = 1;
        }
    }

    // ----------------------------------------------------
    // Trường hợp lệnh không xác định
    // ----------------------------------------------------
    if (!handled) {
        UART_DMA_SendString("STM32: Unknown Command!\r\n");
    }
}
// ==========================================
// THÊM HÀM NÀY VÀO CUỐI FILE message.c
// ==========================================
void MSG_SendRobotState(int16_t ls, int16_t rs, uint8_t state) {
    // Biến static lưu thời điểm gửi cuối cùng
    static uint32_t last_send_time = 0;

    // Giới hạn tần suất gửi (ví dụ mỗi 50ms tương đương 20 khung hình/giây)
    if (HAL_GetTick() - last_send_time >= 50) {
        char buf[64];

        // Đóng gói dữ liệu. ESP32 sẽ nhận được ví dụ: "RSTATE:0,70,70\r\n"
        snprintf(buf, sizeof(buf), "RSTATE:%d,%d,%d\r\n", state, ls, rs);

        // Gọi hàm truyền UART có sẵn của bạn
        UART_DMA_SendString(buf);

        // Cập nhật lại thời gian
        last_send_time = HAL_GetTick();
    }
}


/* ĐỊNH NGHĨA CÁC HÀNH ĐỘNG CHI TIẾT */
void Action_MoveForward(void) {
	Motor_Set(speed, speed - MOTOR_TRIM);
}

void Action_MoveBackward(void) {
	Motor_Set(-speed, -speed + MOTOR_TRIM);
}

void Action_TurnLeft(void) {
    Motor_Set(-speed, speed - MOTOR_TRIM);
}

void Action_TurnRight(void) {
    Motor_Set(speed, -speed + MOTOR_TRIM);
}

void Action_Stop(void) {
    Motor_Stop();
}

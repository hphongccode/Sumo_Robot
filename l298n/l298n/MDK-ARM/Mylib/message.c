
#include "message.h"
#include "uart.h" // Sử dụng lại thư viện gửi dữ liệu đã tạo ở bước trước
#include "Robot_Control.h"

extern float speed_multiplier;
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
    if (strstr(msg, "MOVE_FWD") != NULL) {
        Action_MoveForward();
        UART_DMA_SendString("STM32: Moving Forward...\r\n");
    }
    // Kiểm tra lệnh lùi
    else if (strstr(msg, "MOVE_BWD") != NULL) {
        Action_MoveBackward();
        UART_DMA_SendString("STM32: Moving Backward...\r\n");
    }
    // ----------------------------------------------------
    // [TEST UART] Kiểm tra lệnh TẮT (PWR:0)
    // ----------------------------------------------------
    else if (strstr(msg, "PWR:0") != NULL) {
        speed_multiplier = 0;

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

//        UART_DMA_SendString("STM32: Power OFF, LED PC13 OFF\r\n");
    }
    // ----------------------------------------------------
    // [TEST UART] Kiểm tra lệnh BẬT (PWR:1)
    // ----------------------------------------------------
    else if (strstr(msg, "PWR:1") != NULL) {
        speed_multiplier = 1;

        // BẬT LED
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

//        UART_DMA_SendString("STM32: Power ON, LED PC13 ON\r\n");
    }
    // Trường hợp lệnh không xác định
    else {
        UART_DMA_SendString("STM32: Unknown Command!\r\n");
    }
}

/* ĐỊNH NGHĨA CÁC HÀNH ĐỘNG CHI TIẾT */

void Action_MoveForward(void) {
    // Viết code điều khiển chân PWM cho motor tiến ở đây
    // Ví dụ: HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_PIN, 1);
}

void Action_MoveBackward(void) {
    // Viết code điều khiển motor lùi
}

void Action_Stop(void) {
    // Viết code dừng motor
}

#include "uart.h"
#include <string.h>

extern UART_HandleTypeDef huart1;

uint8_t rx_buffer[RX_BUF_SIZE];      // Buffer cho DMA nhận dữ liệu
uint8_t main_buffer[RX_BUF_SIZE];    // Buffer để xử lý ở vòng lặp chính
uint16_t rx_data_len = 0;
uint8_t data_ready_flag = 0;

void UART_DMA_Init(void) {
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        // 1. Chống tràn Buffer
        if (Size >= RX_BUF_SIZE) {
            Size = RX_BUF_SIZE - 1;
        }

        // 2. Chép dữ liệu
        memcpy(main_buffer, rx_buffer, Size);

        // 3. THÊM CHỐT CHẶN CHUỖI: Rất quan trọng để dùng hàm strstr() an toàn!
        main_buffer[Size] = '\0';

        rx_data_len = Size;
        data_ready_flag = 1;

        // 4. Khởi động lại DMA
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}

void UART_DMA_SendString(char *str) {
    // CHỐNG GHI ĐÈ: Chờ đến khi UART rảnh rỗi mới cho gửi tiếp
	HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
}

void UART_DMA_SendData(uint8_t *data, uint16_t len) {
    // CHỐNG GHI ĐÈ
	HAL_UART_Transmit(&huart1, data, len, 100);
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // Nếu UART báo lỗi (như Overrun), ép nó xóa lỗi và tiếp tục nghe
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}

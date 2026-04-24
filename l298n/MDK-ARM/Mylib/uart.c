#include "uart.h"

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
        // Sao chép dữ liệu sang bộ đệm chính để xử lý, tránh bị DMA ghi đè
        memcpy(main_buffer, rx_buffer, Size);
        rx_data_len = Size;
        data_ready_flag = 1;

        // Quan trọng: Khởi động lại chế độ nhận DMA cho gói tin tiếp theo
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}


void UART_DMA_SendString(char *str) {
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)str, strlen(str));
}

void UART_DMA_SendData(uint8_t *data, uint16_t len) {
    HAL_UART_Transmit_DMA(&huart1, data, len);
}

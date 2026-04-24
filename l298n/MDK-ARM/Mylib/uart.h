

#ifndef UART_H_
#define UART_H_

#include "stm32f1xx_hal.h"
#include <string.h>


#define RX_BUF_SIZE 256

extern uint8_t rx_buffer[RX_BUF_SIZE];
extern uint8_t main_buffer[RX_BUF_SIZE];
extern uint16_t rx_data_len;
extern uint8_t data_ready_flag;

void UART_DMA_Init(void);
void UART_DMA_SendString(char *str);
void UART_DMA_SendData(uint8_t *data, uint16_t len);

#endif /* SRC_UART_H_ */

//UART_DMA_Init();
//UART_DMA_SendString("STM32 Ready to talk with ESP32\r\n");
///* USER CODE END 2 */
//
//while (1) {
//    if (data_ready_flag) {
//        UART_DMA_SendString("Received: ");
//        UART_DMA_SendData(main_buffer, rx_data_len);
//        data_ready_flag = 0;
//        memset(main_buffer, 0, RX_BUF_SIZE);
//    }
//}

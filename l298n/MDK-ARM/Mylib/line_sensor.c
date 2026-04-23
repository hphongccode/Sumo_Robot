#include <stdio.h>
#include "line_sensor.h"


/* ===== GLOBAL ===== */
volatile uint8_t line_flag = 0;
volatile uint8_t line_dir  = 0;
volatile uint32_t line_time = 0;

/* ===== INIT ===== */
void Line_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* C?u hình các chân c?m bi?n Line: PB0 (Front), PB1 (Back), PB10 (Left), PB11 (Right) */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;   // LOW trigger khi phát hi?n v?ch
    GPIO_InitStruct.Pull = GPIO_PULLUP;            // R?t quan tr?ng d? tránh nhi?u
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* C?u hình uu tiên ng?t (NVIC) */
    HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* ===== INTERRUPT SERVICE ROUTINE (ISR) ===== */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(!(GPIO_Pin & (GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11))) return;

    /* Ð?c TOÀN B? tr?ng thái ngay l?p t?c – b?t t?t c? sensor dù ch? 1 pin trigger ISR */
    uint8_t mask = 0;
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)  == GPIO_PIN_RESET) mask |= LINE_FRONT;
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)  == GPIO_PIN_RESET) mask |= LINE_BACK;
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET) mask |= LINE_LEFT;
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET) mask |= LINE_RIGHT;

    if(mask == 0) return;   // Noise: pin dã v? HIGH tru?c khi d?c k?p

    line_dir  |= mask;      // OR accumulate – không m?t sensor nào k? c? trigger l?ch th?i gian
    line_flag  = 1;
    line_time  = HAL_GetTick();
}

/* ===== READ ===== */
LineState Line_Read(void)
{
    LineState s;
    s.front = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)  == GPIO_PIN_RESET);
    s.back  = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)  == GPIO_PIN_RESET);
    s.left  = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET);
    s.right = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET);
    uint8_t sum = s.front + s.back + s.left + s.right;
    if(sum > 3) { s.front = s.back = s.left = s.right = 0; }
    return s;
}
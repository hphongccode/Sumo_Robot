#include <stdio.h>
#include "line_sensor.h"
/* LUU Ý: Không c?n include "Motor.h" hay g?i extern các bi?n tr?ng thái t? main n?a */

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
    uint32_t now = HAL_GetTick();
    static uint32_t last_exti_time = 0;

    /* 1. Debounce siêu ng?n (10ms): 
       Ch? l?c nhi?u v?t lý (bouncing) c?a quang tr?/m?t h?ng ngo?i, 
       không làm ch?n các c?m bi?n khác kích ho?t liên ti?p. */
    if(now - last_exti_time < 10) return; 
    last_exti_time = now;

   
    if(GPIO_Pin == GPIO_PIN_0)       line_dir = 1; // Front
    else if(GPIO_Pin == GPIO_PIN_1)  line_dir = 2; // Back
    else if(GPIO_Pin == GPIO_PIN_10) line_dir = 3; // Left
    else if(GPIO_Pin == GPIO_PIN_11) line_dir = 4; // Right
    else return; 

    /* 3. B?t c? báo cáo cho main loop x? lý */
    line_flag = 1;
    line_time = now;
}

/* ===== READ ===== */
LineState Line_Read(void)
{
    LineState s;

    
    s.front = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET);
    s.back  = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET);
    s.left  = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET);
    s.right = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET);

    /* ===== FILTER =====  */
    uint8_t sum = s.front + s.back + s.left + s.right;

    if(sum >= 3)
    {
        s.front = 0;
        s.back  = 0;
        s.left  = 0;
        s.right = 0;
    }

    return s;
}
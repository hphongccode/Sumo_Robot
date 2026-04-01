#include "line_sensor.h"
#include <stdio.h>

/* INIT */
void Line_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* L?c nhi?u */
static uint8_t stable_read(GPIO_TypeDef* port, uint16_t pin)
{
    uint8_t cnt = 0;

    for(int i = 0; i < 5; i++)
    {
        if(HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET)
            cnt++;
    }

    return (cnt >= 3);
}

/* READ */
LineState Line_Read(void)
{
    LineState s;

    s.front = stable_read(GPIOB, GPIO_PIN_0);
    s.back  = stable_read(GPIOB, GPIO_PIN_1);
    s.left  = stable_read(GPIOB, GPIO_PIN_10);
    s.right = stable_read(GPIOB, GPIO_PIN_11);

    return s;
}
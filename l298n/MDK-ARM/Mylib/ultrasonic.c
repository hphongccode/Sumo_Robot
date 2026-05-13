#include "ultrasonic.h"

extern TIM_HandleTypeDef htim2;

/* delay microsecond */
static void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while(__HAL_TIM_GET_COUNTER(&htim2) < us);
}

/* Init */
void Ultra_Init(void)
{
    HAL_TIM_Base_Start(&htim2);
}

/* Read single sensor */
float Ultra_Read(GPIO_TypeDef* trig_port, uint16_t trig_pin,
                 GPIO_TypeDef* echo_port, uint16_t echo_pin)
{
    uint32_t time = 0;
    uint32_t timeout = 30000;

    /* TRIG pulse */
    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);

    /* wait echo HIGH */
    while(HAL_GPIO_ReadPin(echo_port, echo_pin) == GPIO_PIN_RESET)
    {
        if(timeout-- == 0) return -1;
    }

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    timeout = 30000;

    /* measure */
    while(HAL_GPIO_ReadPin(echo_port, echo_pin) == GPIO_PIN_SET)
    {
        time = __HAL_TIM_GET_COUNTER(&htim2);
        if(timeout-- == 0) break;
    }

    return time * 0.034 / 2;
}

/* Read all sensors (có delay tránh nhi?u chéo) */
UltraState Ultra_ReadAll(void)
{
    UltraState u;

    /* LEFT */
    u.left = Ultra_Read(GPIOA, GPIO_PIN_1,
                        GPIOA, GPIO_PIN_0);
    HAL_Delay(10);

    /* MID */
    u.mid  = Ultra_Read(GPIOA, GPIO_PIN_3,
                        GPIOA, GPIO_PIN_2);
    HAL_Delay(10);

    /* RIGHT */
    u.right= Ultra_Read(GPIOA, GPIO_PIN_5,
                        GPIOA, GPIO_PIN_4);
    HAL_Delay(10);

    return u;
}
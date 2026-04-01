#include "motor.h"

extern TIM_HandleTypeDef htim3;

/* =========================
   Internal Speed Control
   ========================= */

static void Motor_SetSpeedLeft(uint8_t speed)
{
    if(speed > MOTOR_MAX_SPEED)
        speed = MOTOR_MAX_SPEED;

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
}

static void Motor_SetSpeedRight(uint8_t speed)
{
    if(speed > MOTOR_MAX_SPEED)
        speed = MOTOR_MAX_SPEED;

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
}



void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    Motor_Stop();
}

/* LEFT SIDE (2 bánh trái)*/
/*
PA6	ENA
PB4	IN1
PB5	IN2
PA7	ENB
PB6	IN3
PB7	IN4
*/
void Motor_LeftForward(uint8_t speed)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

    Motor_SetSpeedLeft(speed);
}

void Motor_LeftBackward(uint8_t speed)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

    Motor_SetSpeedLeft(speed);
}

/* 
   RIGHT SIDE (2 bánh phai)
*/

void Motor_RightForward(uint8_t speed)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

    Motor_SetSpeedRight(speed);
}

void Motor_RightBackward(uint8_t speed)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

    Motor_SetSpeedRight(speed);
}

/* 
   ROBOT MOVEMENT
*/

void Motor_Forward(uint8_t speed)
{
    Motor_LeftForward(speed);
    Motor_RightForward(speed);
}

void Motor_Backward(uint8_t speed)
{
    Motor_LeftBackward(speed);
    Motor_RightBackward(speed);
}

void Motor_Left(uint8_t speed)
{
    Motor_LeftBackward(speed);
    Motor_RightForward(speed);
}

void Motor_Right(uint8_t speed)
{
    Motor_LeftForward(speed);
    Motor_RightBackward(speed);
}

void Motor_Stop(void)
{
    Motor_SetSpeedLeft(0);
    Motor_SetSpeedRight(0);
}
void Motor_Set(int16_t left, int16_t right)
{
    if(left >= 0)
        Motor_LeftForward(left);
    else
        Motor_LeftBackward(-left);

    if(right >= 0)
        Motor_RightForward(right);
    else
        Motor_RightBackward(-right);
}
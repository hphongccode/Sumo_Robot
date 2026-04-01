#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

#define MOTOR_MAX_SPEED 100

void Motor_Init(void);

/* Control tung ben */
void Motor_LeftForward(uint8_t speed);
void Motor_LeftBackward(uint8_t speed);

void Motor_RightForward(uint8_t speed);
void Motor_RightBackward(uint8_t speed);

/* Control robot */
void Motor_Forward(uint8_t speed);
void Motor_Backward(uint8_t speed);

void Motor_Left(uint8_t speed);
void Motor_Right(uint8_t speed);

void Motor_Stop(void);
void Motor_Set(int16_t left, int16_t right);
#endif
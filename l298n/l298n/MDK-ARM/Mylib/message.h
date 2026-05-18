/*
 * message.h
 *
 *  Created on: Apr 24, 2026
 *      Author: ASUS
 */

#ifndef MESSAGE_H_
#define MESSAGE_H_

#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>

/* Kích thước tối đa của một câu lệnh */
#define MAX_CMD_LEN 64

/* Nguyên mẫu hàm */
void CMD_Process(uint8_t *data, uint16_t len);

/* Các hàm hành động (Ví dụ cho Robot Sumo) */
void Action_MoveForward(void);
void Action_MoveBackward(void);
void Action_Stop(void);
void Action_ToggleLED(void);

#endif /* MESSAGE_H_ */

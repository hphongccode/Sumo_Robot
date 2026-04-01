#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include "main.h"

typedef struct {
    uint8_t front;
    uint8_t back;
    uint8_t left;
    uint8_t right;
} LineState;

void Line_Init(void);
LineState Line_Read(void);

#endif
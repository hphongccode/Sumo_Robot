#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include "main.h"
#define LINE_FRONT  (1 << 0)   // 0x01
#define LINE_BACK   (1 << 1)   // 0x02
#define LINE_LEFT   (1 << 2)   // 0x04
#define LINE_RIGHT  (1 << 3)   // 0x08

typedef struct {
    uint8_t front;
    uint8_t back;
    uint8_t left;
    uint8_t right;
} LineState;

void Line_Init(void);
LineState Line_Read(void);

/* interrupt flag */
extern volatile uint8_t line_flag;
extern volatile uint8_t line_dir;
extern volatile uint32_t line_time;
#endif
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "main.h"

typedef struct {
    float left;
    float mid;
    float right;
} UltraState;

/* Init */
void Ultra_Init(void);

/* Read single */
float Ultra_Read(GPIO_TypeDef* trig_port, uint16_t trig_pin,
                 GPIO_TypeDef* echo_port, uint16_t echo_pin);

/* Read all */
UltraState Ultra_ReadAll(void);

#endif
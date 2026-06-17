
#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f4xx_hal.h"

void Servo_Init(TIM_HandleTypeDef *htim, uint32_t channel);
void Servo_WriteAngle(uint16_t angle);

#endif /* __SERVO_H */


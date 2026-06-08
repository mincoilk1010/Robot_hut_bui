/*
 * motor.h
 *
 *  Created on: Jun 7, 2026
 *      Author: GB Center
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_
#include "types.h"
#include "main.h"
extern volatile int8_t Motor_Left_Dir;
extern volatile int8_t Motor_Right_Dir;
extern TIM_HandleTypeDef htim4;
void motor_pwm_init(u8 psc, u16 arr); // TIMER 4
void motor_r_stop();
void motor_l_stop();
void motor_run(i16 left_pwm, i16 right_pwm);
void motor_init(u8 psc, u16 arr);

#endif /* INC_MOTOR_H_ */

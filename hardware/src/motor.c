/*
 * motor.c
 *
 *  Created on: Jun 7, 2026
 *      Author: GB Center
 */

#include "motor.h"

void motor_pwm_init(u8 psc, u16 arr)
{
	TIM4->PSC = psc;
	TIM4->ARR = arr;
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);

}
void motor_l_stop()
{
	TIM4->CCR1 = 0;
	TIM4->CCR2 = 0;

}
void motor_r_stop()
{
	TIM4->CCR3 = 0;
	TIM4->CCR4 = 0;
}
void motor_stop()
{
	Motor_Left_Dir = 0;
	Motor_Right_Dir = 0;

	motor_r_stop();
	motor_l_stop();
}

void motor_run(i16 left_pwm, i16 right_pwm)
{
    left_pwm = limit(left_pwm,999,-999);
    right_pwm = limit(right_pwm,999,-999);

    if (left_pwm > 0)
    {
        Motor_Left_Dir = 1;
        TIM4->CCR1 = left_pwm;
        TIM4->CCR2 = 0;
    }
    else if (left_pwm < 0)
    {
        Motor_Left_Dir = -1;
        TIM4->CCR1 = 0;
        TIM4->CCR2 = -left_pwm;
    }
    else
    {
        Motor_Left_Dir = 0;
        TIM4->CCR1 = 0;
        TIM4->CCR2 = 0;
    }

    if (right_pwm > 0)
    {
        Motor_Right_Dir = 1;
        TIM4->CCR3 = right_pwm;
        TIM4->CCR4 = 0;
    }
    else if (right_pwm < 0)
    {
        Motor_Right_Dir = -1;
        TIM4->CCR3 = 0;
        TIM4->CCR4 = -right_pwm;
    }
    else
    {
        Motor_Right_Dir = 0;
        TIM4->CCR3 = 0;
        TIM4->CCR4 = 0;
    }
}
void motor_init(u8 psc, u16 arr)
{
	motor_pwm_init(psc, arr);
	motor_stop();

}

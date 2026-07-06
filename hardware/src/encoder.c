/*
 * encoder.c
 *
 *  Created on: Jun 7, 2026
 *      Author: GB Center
 */
#include "encoder.h"
#include "motor.h"
Encoder_data_t ec_l;
Encoder_data_t ec_r;
_vo i8 Motor_Left_Dir;
_vo i8 Motor_Right_Dir;

#define ENC_STEP_M         ((PI * WHEEL_DIAMETER_M) / ENCODER_PPR)

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	g_ms = HAL_GetTick();

    if(htim->Instance == TIM2)
    {
        uint32_t cap = TIM2->CCR1;
        uint32_t per = cap - ec_l.cap_last;

        ec_l.cap_last = cap;
        ec_l.tick = g_ms;
        ec_l.total++;

        if(per > 100u)
        {
            ec_l.period = per;

            float v = 10210.18f / (float)per;

            if(Motor_Left_Dir < 0)
                ec_l.vel = -v;

            if(Motor_Left_Dir > 0)
                ec_l.vel = v;

            ec_l.rpm = 3000000.0f / (float)per;
            ec_l.active = 1u;
        }

        if(Motor_Left_Dir < 0)
        {
            ec_l.dist -= ENC_STEP_M;
        }

        if(Motor_Left_Dir > 0)
        {
            ec_l.dist += ENC_STEP_M;
        }
    }

    if(htim->Instance == TIM5)
    {
        uint32_t cap = TIM5->CCR2;
        uint32_t per = cap - ec_r.cap_last;

        ec_r.cap_last = cap;
        ec_r.tick = g_ms;
        ec_r.total++;

        if(per > 100u)
        {
            ec_r.period = per;

            float v = 10210.18f / (float)per;

            if(Motor_Right_Dir < 0)
                ec_r.vel = -v;

            if(Motor_Right_Dir > 0)
                ec_r.vel = v;

            ec_r.rpm = 3000000.0f / (float)per;
            ec_r.active = 1u;
        }

        if(Motor_Right_Dir < 0)
        {
            ec_r.dist -= ENC_STEP_M;
        }

        if(Motor_Right_Dir > 0)
        {
            ec_r.dist += ENC_STEP_M;
        }
    }

}

void encoder_init()
{
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim5);

	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
}

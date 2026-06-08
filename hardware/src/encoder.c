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
_vo u32 g_ms;
_vo i8 Motor_Left_Dir;
_vo i8 Motor_Right_Dir;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM3)
    {
        uint32_t cap = TIM3->CCR2;

        uint32_t per = (uint16_t)(cap - (uint16_t)ec_l.cap_last);

        ec_l.cap_last = cap;
        ec_l.tick = g_ms;
        ec_l.total++;

        if(per > 200)
        {
            ec_l.period = per;

            float v = 10210.18f / (float)per;

            if(Motor_Left_Dir < 0)
                ec_l.vel = -v;

            if(Motor_Left_Dir > 0)
                ec_l.vel = v;

            ec_l.rpm = 3000000.0f / (float)per;
            ec_l.active = 1;
        }

        if(Motor_Left_Dir < 0)
        {
            ec_l.dist -= ((PI * WHEEL_DIAMETER_M) / ENCODER_PPR);
        }

        if(Motor_Left_Dir > 0)
        {
            ec_l.dist += ((PI * WHEEL_DIAMETER_M) / ENCODER_PPR);
        }
    }

    if(htim->Instance == TIM5)
    {
        uint32_t cap = TIM5->CCR2;

        uint32_t per = (uint16_t)(cap - (uint16_t)ec_r.cap_last);

        ec_r.cap_last = cap;
        ec_r.tick = g_ms;
        ec_r.total++;

        if(per > 200)
        {
            ec_r.period = per;

            float v = 10210.18f / (float)per;

            if(Motor_Right_Dir < 0)
                ec_r.vel = -v;

            if(Motor_Right_Dir > 0)
                ec_r.vel = v;

            ec_r.rpm = 3000000.0f / (float)per;
            ec_r.active = 1;
        }

        if(Motor_Right_Dir < 0)
        {
            ec_r.dist -= ((PI * WHEEL_DIAMETER_M) / ENCODER_PPR);
        }

        if(Motor_Right_Dir > 0)
        {
            ec_r.dist += ((PI * WHEEL_DIAMETER_M) / ENCODER_PPR);
        }
    }
}

void encoder_init()
{
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
}

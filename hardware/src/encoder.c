/*
 * encoder.c
 *
 *  Created on: Jun 7, 2026
 *      Author: GB Center
 */
#include "encoder.h"
#include "motor.h"
#include "hc_sr04.h"
Encoder_data_t ec_l;
Encoder_data_t ec_r;

_vo i8 Motor_Left_Dir;
_vo i8 Motor_Right_Dir;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	g_ms = HAL_GetTick();
    if(htim->Instance == TIM2)
    {
        u32 cap = TIM2->CCR1;
        u32 per = (u32)(cap - (u32)ec_l.cap_last);

        ec_l.cap_last = cap;
        ec_l.tick = g_ms;
        ec_l.total++;

        if(per > 100)
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
        u32 cap = TIM5->CCR2;

        u32 per = (u32)(cap - (u32)ec_r.cap_last);

        ec_r.cap_last = cap;
        ec_r.tick = g_ms;
        ec_r.total++;

        if(per > 100)
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

	if(htim->Instance == TIM1)
	{

		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 && hc[0].busy)
		{
			
			 if(!hc[0].first_cap)
			 {
				hc[0].val1 = TIM1->CCR1;
				hc[0].first_cap = 1;
				TIM1->CCER |= (1 << 1); // falling
			 }
			 else
			 {
				hc[0].val2 = TIM1->CCR1;
				if(hc[0].val2 > hc[0].val1)
				{
					hc[0].diff = hc[0].val2 - hc[0].val1;
				}else
				{
					hc[0].diff = 0xFFFF - hc[0].val1 + hc[0].val2;
				}

				if(hc[0].diff > 25000)
				{
					hc[0].d = 9999;
				}
				else{
					// d = time * 0.17
					// v am thanh = 343 m/s = 0.343 mm/us
					hc[0].d = (u16)(hc[0].diff * 17 / 100);

				}
				hc[0].done = 1; // bao timer do xong
				hc[0].busy = 0;
				hc[0].first_cap = 0;
				hc[0].sample_ms = HAL_GetTick();
				TIM1->CCER &= ~(1 << 1); //  falling edge
				TIM1->DIER &= ~(1 << 1);
				

		}

		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2 && hc[1].busy)
		{
			if(!hc[1].first_cap)
			{
				hc[1].val1 = TIM1->CCR2;
				hc[1].first_cap = 1;
				TIM1->CCER |= (1 << 5); /* CC2P: capture falling edge */
			}
			else
			{
				hc[1].val2 = TIM1->CCR2;
				if(hc[1].val2 > hc[1].val1)
				{
					hc[1].diff = hc[1].val2 - hc[1].val1;

				}
				else{
					hc[1].diff = 0xFFFF - hc[1].val1 + hc[1].val2;
				}
				if(hc[1].diff > 25000)
				{
					hc[1].d = 9999;
				}
				else{

					hc[1].d = (u16)(hc[1].diff * 17 / 100);
				}
				hc[1].done = 1;
				hc[1].busy = 0;
				hc[1].first_cap = 0;
				hc[1].sample_ms = HAL_GetTick();
				TIM1->CCER &= ~(1<<5); /* CC2P: back to rising edge */
				TIM1->DIER &= ~(1 << 2);
				
			}
		}

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

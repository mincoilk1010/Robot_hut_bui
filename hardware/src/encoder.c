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
        uint32_t cap = TIM2->CCR1;
        uint32_t per = (u32)(cap - (u32)ec_l.cap_last);

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
        uint32_t cap = TIM5->CCR2;

        uint32_t per = (u32)(cap - (u32)ec_r.cap_last);

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

		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
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

				if(hc[0].diff > 1400)
				{
					hc[0].d = 9999;
				}
				else{
					// d = time * 0.17
					// v am thanh = 343 m/s = 0.343 mm/us
					hc[0].d = (u16)(hc[0].diff * 17 / 100);

				}
				hc[0].done = 1; // bao timer do xong
				hc[0].first_cap = 0;
				TIM1->CCER &= ~(1 << 1); //  falling edge
				TIM1->DIER &= ~(1 << 1); // dis intrerrupt

			 }
		}


		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			if(!hc[1].first_cap)
			{
				hc[1].val1 = TIM1->CCR2;
				hc[1].first_cap = 1;
				TIM1->CCER |= (1 << 2);
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
				if(hc[1].diff > 1400)
				{
					hc[1].d = 9999;
				}
				else{

					hc[1].d = (u16)(hc[1].diff * 17 / 100);
				}
				hc[1].done = 1;
				hc[1].first_cap = 0;
				TIM1->CCER &= ~(1<<2);
				TIM1->DIER &= ~(1<<2);
			}


		}

		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		{
			if(!hc[2].first_cap)
			{
				hc[2].first_cap = 1;
				hc[2].val1 = TIM1->CCR3;
				TIM1->CCER |= (1<<3);
			}
			else
			{
				hc[2].val2 = TIM1->CCR3;
				if(hc[2].val2 > hc[2].val1)
				{
					hc[2].diff = hc[2].val2 - hc[2].val1;
				}
				else
				{
					hc[2].diff = 0xFFFF - hc[2].val1 + hc[2].val2;
				}

				if(hc[2].diff > 1400)
				{
					hc[2].d = 9999;
				}
				else
				{
					hc[2].d = (u16)(hc[2].diff * 17 / 100);
				}
				hc[0].done = 1;
				hc[0].first_cap = 0;
				TIM1->CCER &= ~(1 << 3);
				TIM1->DIER &= ~(1 << 3);
			}


		}

		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
		{
			if(!hc[3].first_cap)
			{
				hc[3].val1 = TIM1->CCR4;
				hc[3].first_cap = 1;
				TIM1->CCER |= (1 << 2);

			}
			else
			{
				hc[3].val2 = TIM1 -> CCR4;
				if(hc[3].val2 > hc[3].val1)
				{
					hc[3].diff = hc[3].val2 - hc[3].val1;
				}
				else{
					hc[3].diff = 0xFFFF - hc[3].val1 + hc[3].val2;

				}

				if(hc[3].diff > 1400)
				{
					hc[3].d = 9999;
				}
				else{
					hc[3].d = (u16)(hc[3].diff * 17/100);
				}
				hc[3].done = 1;
				hc[3].first_cap = 0;
				TIM1->CCER &= ~(1 << 4);
				TIM1->CCER &= ~(1 << 4);
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

/*
 * hc_sr04.c
 *
 *  Created on: May 14, 2026
 *      Author: GB Center
 */

#include "hc_sr04.h"
#include "delay.h"

HC_t hc[2] ={0};

void hcsr04_read()
{
	for (u8 i=0; i < 4; i++)
	{
		hc[i].first_cap = 0;
		hc[i].done = 0;
	}
	TIM1->SR = 0;

	TIM1->CCER &= ~((1 << 1) | (1 << 5) | (1 << 9) | (1 << 13));

	TIM1->DIER |= (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4);// enable
    //set HIGH, resigter BSRR
	GPIOD->BSRR = (1 << 8) | (1 << 9) | (1 << 10) | (1 << 11);
	delay_us(10);
	//set Low
	GPIOD->BSRR = (1 << 24) | (1 << 25) | (1 << 26) | (1 << 27);
	
}
void hcsr04_init()
{
	hc[0].f=0;
	hc[1].f=0;
	hc[0].d=0;
	hc[1].d=0;
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
}

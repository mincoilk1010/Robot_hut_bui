/*
 * delay.c
 *
 *  Created on: Jun 8, 2026
 *      Author: GB Center
 */


/*
 * delay.c
 *
 *  Created on: May 6, 2026
 *      Author: GB Center
 */


#include "delay.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "encoder.h"
//#include "encoder.h"
#define TIMER3 1
#define SYS_TICK 2
#define DELAY_SRC TIMER3
//#define DELAY_SRC SYS_TICK
#
#define TIM3_BASEADDR (0x40000400U)
#define TIM3_CLK_MHz 84
#define SYSCLK_MHz 168
#if DELAY_SRC != TIMER3 && DELAY_SRC != SYS_TICK
#error DELAY_SRC must be TIMER2 or SYS_TICK
#endif
u32 tim1_cnt = 0;

#if(DELAY_SRC == SYS_TICK)

_vo u32 systick_ms = 0;
static u32 fac_us = 0;
#endif
void delay_init()
{
#if (DELAY_SRC == TIMER3)
	//set 1 SEC for timer
	//rcc-->16Mhz->>psc(16)->>1000000Hz/1cnt ->1us
	//ARR : 1000


	//TIM2_PCLK_EN();
	__HAL_RCC_TIM3_CLK_ENABLE();

	u32* CR1 = (u32*)(TIM3_BASEADDR + 0x00);
	u32* ARR = (u32*)(TIM3_BASEADDR + 0x2C);
	u32* PSC = (u32*)(TIM3_BASEADDR + 0x28);
	//u32* DIER = (u32*)(TIM2_BASEADDR + 0X0C);
	u32* CNT = (u32*)(TIM3_BASEADDR + 0X24);
	u32* EGR = (u32*)(TIM3_BASEADDR + 0x14);
	*CR1 &= ~(1 << 0);

	*ARR = 0xffff;
	*PSC = TIM3_CLK_MHz - 1;
	*CNT = 0;

	*EGR = 1; // Update generation

	//*DIER |= 1 << 0;  // enable interrupt
	*CR1 |= 1 << 0;  //enable counter

	//u32 *ISER0 = (u32*)(0xe000e100);

	//*ISER0 |= 1 << 25;

#else
	u32 *CSR = (u32*)0xE000E010;
	u32 *RVR = (u32*)0xE000E014;
	u32 *CVR = (u32*)0xE000E018;
	//u32 *CVR = (u32*)0xE000E018;

	fac_us = SYSCLK_MHz;

	*CSR = 0; //
	*RVR = (SYSCLK_MHz * 1000U) - 1;


	*CVR = 0; // clean current counter
	*CSR = (1<<0) | (1<<1) | (1<<2);
#endif

}


/*
void TIM1_UP_TIM10_IRQHandler()
{

	u16 *SR =(u16*)(0x40010010);
	*SR &= ~(1 << 0);
	tim1_cnt++;
}
*/
/*
void tim1_delay_1sec()
{
	u16 *SR = (u16*)(0x40010010);
	while(((*SR << 0) & 1) != 1); //Set UIF flag is set to 1

	*SR &= ~(1<<0);   // clean UIF flag


}

*/
/*
#if(DELAY_SRC == SYS_TICK)

void SysTick_Handler(void)
{
	HAL_IncTick();
	systick_ms++;
	g_ms++;

}

#endif
*/
void delay_us(u32 us)
{
#if (DELAY_SRC == TIMER3)
	u32 *CNT = (u32*)(TIM3_BASEADDR + 0x24);

	u32 start = *CNT;
	while((u16)((u16)*CNT -start) < (u16)us);
#else
	u32 temp;
	u32 ticks;
	u32 *CSR = (u32 *)0xE000E010;
	u32 *RVR = (u32 *)0xE000E014;
	u32 *CVR = (u32 *)0xE000E018;


	if(us == 0)
	{
		return;
	}

	ticks = us * fac_us;

	*CSR = 0;

	while(ticks > 0xfffff)
	{
		*RVR = 0xfffff;
		*CVR = 0;


		*CSR = (1 << 0) | (1 << 2);

		do
		{
			temp = *CSR;
		}
		while((temp & 0x01) && !(temp & (1 << 16)));

		*CSR = 0;
		*CVR = 0;
	}

	if(ticks > 0)
	{
		*RVR = ticks - 1;
		*CVR = 0;

		/*
		 * Bật SysTick polling, không dùng interrupt
		 */
		*CSR = (1 << 0) | (1 << 2);

		do
		{
			temp = *CSR;
		}
		while((temp & 0x01) && !(temp & (1 << 16)));

		*CSR = 0;
		*CVR = 0;
	}

	/*
	 * Khôi phục lại SysTick interrupt mỗi 1ms.
	 */
	*RVR = (SYSCLK_MHz * 1000U) - 1U;
	*CVR = 0;
	*CSR = (1 << 0) | (1 << 1) | (1 << 2);
#endif
}
void delay_ms(u32 mssec)
{
#if (DELAY_SRC == TIMER3)

	HAL_Delay(mssec);

#else

	u32 current_cnt = systick_ms;

	while((u32)(systick_ms - current_cnt) < mssec);

#endif
}

u32 millis(void)
{
#if (DELAY_SRC == TIMER3)


	return HAL_GetTick();

#else

	return systick_ms;

#endif
}

u32 micros(void)
{
#if (DELAY_SRC == TIMER3)

	u32 *CNT = (u32 *)(TIM3_BASEADDR + 0x24);
	return *CNT;

#else

	return systick_ms * 1000U;

#endif
}



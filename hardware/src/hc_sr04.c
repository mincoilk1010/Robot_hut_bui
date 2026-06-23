
#include "hc_sr04.h"
#include "delay.h"

HC_t hc[2] ={0};

void hcsr04_read()
{
	/* Không bắn trigger mới nếu vòng đo trước chưa xong, tránh xung
	 * chồng lấp làm sai capture. */
	if(hc[0].busy || hc[1].busy) return;

	u32 now = HAL_GetTick();
	for (u8 i=0; i < 2; i++)
	{
		hc[i].first_cap = 0;
		hc[i].done = 0;
		hc[i].busy = 1;
		hc[i].start_ms = now;
	}
	TIM1->DIER |= (1 << 1);  /* CC1IE */
    TIM1->DIER |= (1 << 2);  /* CC2IE */
    TIM1->CCER &= ~(1 << 1); /* dam bao dang o rising cho CH1 (CC1P = 0) */
    TIM1->CCER &= ~(1 << 5); /* dam bao dang o rising cho CH2 (CC2P = 0) */
    //set HIGH, resigter BSRR
	GPIOE->BSRR = (1 << 10) | (1 << 12) ;
	delay_us(10);
	//set Low
	GPIOE->BSRR = (1 << 26) | (1 << 28) ;
	
}
void hcsr04_check_timeout(void)
{
    uint32_t now = HAL_GetTick();
 
    for (u8 i = 0; i < 2; i++)
    {
        if (hc[i].busy && !hc[i].done)
        {
            if ((u32)(now - hc[i].start_ms) > 60u) /* 60ms ~ du cho echo toi da ~4m */
            {
                hc[i].d         = 9999;
                hc[i].done      = 1;
                hc[i].busy      = 0;
                hc[i].first_cap = 0;
                /* dam bao interrupt khong bi ket o trang thai cho falling */
                if (i == 0)
                {
                    TIM1->CCER &= ~(1 << 1);
                }
                else
                {
                    TIM1->CCER &= ~(1 << 5);
                }
            }
        }
    }
}
void hcsr04_init()
{
	
	hc[0].d=0;
	hc[1].d=0;
	hc[0].busy = 0;
	hc[1].busy = 0;
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
}

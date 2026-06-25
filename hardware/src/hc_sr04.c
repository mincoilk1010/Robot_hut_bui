#include "hc_sr04.h"
#include "delay.h"

HC_t hc[2] = {0};
static u8 next_sensor = 0u;

void hcsr04_read(void)
{
    /* Fire one module at a time so the two ultrasonic receivers cannot
     * mistake the other module's pulse for their own echo. */
    if (hc[0].busy || hc[1].busy) return;

    u8 i = next_sensor;
    next_sensor ^= 1u;

    hc[i].first_cap = 0u;
    hc[i].done = 0u;
    hc[i].busy = 1u;
    hc[i].start_ms = HAL_GetTick();

    TIM1->DIER &= ~((1u << 1) | (1u << 2));
    TIM1->SR &= ~((1u << 1) | (1u << 2));

    if (i == 0u) {
        TIM1->CCER &= ~(1u << 1);       /* CH1: rising edge */
        TIM1->DIER |=  (1u << 1);       /* CC1IE */
        GPIOE->BSRR = (1u << 10);
        delay_us(10);
        GPIOE->BSRR = (1u << 26);
    } else {
        TIM1->CCER &= ~(1u << 5);       /* CH2: rising edge */
        TIM1->DIER |=  (1u << 2);       /* CC2IE */
        GPIOE->BSRR = (1u << 12);
        delay_us(10);
        GPIOE->BSRR = (1u << 28);
    }
}

void hcsr04_check_timeout(void)
{
    u32 now = HAL_GetTick();

    for (u8 i = 0u; i < 2u; i++) {
        if (hc[i].busy && !hc[i].done &&
            (u32)(now - hc[i].start_ms) > 60u) {
            hc[i].d = 9999u;
            hc[i].done = 1u;
            hc[i].busy = 0u;
            hc[i].first_cap = 0u;
            hc[i].sample_ms = now;

            if (i == 0u) {
                TIM1->CCER &= ~(1u << 1);
                TIM1->DIER &= ~(1u << 1);
            } else {
                TIM1->CCER &= ~(1u << 5);
                TIM1->DIER &= ~(1u << 2);
            }
        }
    }
}

void hcsr04_init(void)
{
    hc[0].d = 0u;
    hc[1].d = 0u;
    hc[0].busy = 0u;
    hc[1].busy = 0u;
    hc[0].sample_ms = 0u;
    hc[1].sample_ms = 0u;
    next_sensor = 0u;

    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
    TIM1->DIER &= ~((1u << 1) | (1u << 2));
}

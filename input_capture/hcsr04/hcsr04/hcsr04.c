#include "hcsr04.h"
#include <stddef.h>

/**
  */
void HCSR04_Cliff_Init(HCSR04_Cliff_t* dev, GPIO_TypeDef* trig_port, uint16_t trig_pin,
                       TIM_HandleTypeDef* htim, uint32_t channel, float limit_dist,
                       void (*callback_func)(HCSR04_Cliff_t*))
{
    dev->TRIG_Port      = trig_port;
    dev->TRIG_Pin       = trig_pin;
    dev->htim           = htim;
    dev->TIM_Channel    = channel;
    dev->limit_distance = limit_dist;
    dev->Cliff_Callback = callback_func;
    dev->state          = HCSR04_IDLE_STATE;
    dev->distance       = 0.0f;
    dev->filter_count   = 0;

    HAL_TIM_IC_Start_IT(dev->htim, dev->TIM_Channel);
}

void HCSR04_Cliff_Start(HCSR04_Cliff_t* dev)
{
    if (dev->state == HCSR04_IDLE_STATE)
    {
        __HAL_TIM_SET_CAPTUREPOLARITY(dev->htim, dev->TIM_Channel, TIM_INPUTCHANNELPOLARITY_RISING);

        HAL_GPIO_WritePin(dev->TRIG_Port, dev->TRIG_Pin, GPIO_PIN_SET);
        HAL_Delay(15);
        HAL_GPIO_WritePin(dev->TRIG_Port, dev->TRIG_Pin, GPIO_PIN_RESET);

        dev->state = HCSR04_WAIT_RISING_STATE;
    }
}

void HCSR04_Cliff_Capture_Callback(HCSR04_Cliff_t* dev, TIM_HandleTypeDef* htim)
{
    if (htim->Instance == dev->htim->Instance)
    {
        if (htim->Channel == (HAL_TIM_ACTIVE_CHANNEL_1 << (dev->TIM_Channel / 4)))
        {
            switch(dev->state)
            {
                case HCSR04_WAIT_RISING_STATE:
                    dev->t_rising = HAL_TIM_ReadCapturedValue(htim, dev->TIM_Channel);
                    dev->state = HCSR04_WAIR_FALLING_STATE;
                    __HAL_TIM_SET_CAPTUREPOLARITY(htim, dev->TIM_Channel, TIM_INPUTCHANNELPOLARITY_FALLING);
                    break;

                case HCSR04_WAIR_FALLING_STATE:
                    dev->t_falling = HAL_TIM_ReadCapturedValue(htim, dev->TIM_Channel);
                    dev->state = HCSR04_COMPLETE_STATE;
                    __HAL_TIM_SET_CAPTUREPOLARITY(htim, dev->TIM_Channel, TIM_INPUTCHANNELPOLARITY_RISING);
                    break;

                default:
                    break;
            }
        }
    }
}

void HCSR04_Cliff_Handle(HCSR04_Cliff_t* dev)
{
    if (dev->state == HCSR04_COMPLETE_STATE)
    {
        uint32_t delta_t = 0;
        if (dev->t_falling >= dev->t_rising) {
            delta_t = dev->t_falling - dev->t_rising;
        } else {
            delta_t = (dev->htim->Instance->ARR - dev->t_rising) + dev->t_falling;
        }

        dev->distance = 0.017f * delta_t;

        if (dev->distance >= dev->limit_distance || dev->distance == 0.0f)
        {
            dev->filter_count++;
            if (dev->filter_count >= 2)
            {
                if (dev->Cliff_Callback != NULL) {
                    dev->Cliff_Callback(dev);
                }
            }
        }
        else
        {
            dev->filter_count = 0; // An toàn -> Reset bộ đếm lọc nhiễu
        }

        dev->state = HCSR04_IDLE_STATE;
    }
}

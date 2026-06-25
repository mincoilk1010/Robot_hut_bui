#include "hcsr04.h"
#include <stddef.h>

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
    dev->start_tick     = 0;

    HAL_TIM_IC_Start_IT(dev->htim, dev->TIM_Channel);
}

void HCSR04_Cliff_Start(HCSR04_Cliff_t* dev)
{
    /* Nếu đang ở giữa 1 chu kỳ đo (chờ rising/falling) mà đã quá thời
     * gian timeout -> không còn hi vọng nhận được capture -> ép về
     * TIMEOUT_STATE để Handle() xử lý, tránh kẹt state machine vĩnh viễn. */
    if ((dev->state == HCSR04_WAIT_RISING_STATE || dev->state == HCSR04_WAIT_FALLING_STATE)
        && (HAL_GetTick() - dev->start_tick > HCSR04_TIMEOUT_MS))
    {
        dev->state = HCSR04_TIMEOUT_STATE;
    }

    if (dev->state == HCSR04_IDLE_STATE)
    {
        __HAL_TIM_SET_CAPTUREPOLARITY(dev->htim, dev->TIM_Channel, TIM_INPUTCHANNELPOLARITY_RISING);

        HAL_GPIO_WritePin(dev->TRIG_Port, dev->TRIG_Pin, GPIO_PIN_SET);
        for (volatile uint16_t i = 0; i < 15000; i++);
        HAL_GPIO_WritePin(dev->TRIG_Port, dev->TRIG_Pin, GPIO_PIN_RESET);

        dev->start_tick = HAL_GetTick();
        dev->state = HCSR04_WAIT_RISING_STATE;
    }
}

void HCSR04_Cliff_Capture_Callback(HCSR04_Cliff_t* dev, TIM_HandleTypeDef* htim)
{
    // 1. Kiểm tra xem ngắt có đúng từ Timer của cảm biến này không
    if (htim->Instance == dev->htim->Instance)
    {
        // 2. Chuyển đổi chuẩn xác TIM_Channel sang dạng Active Channel của HAL
        HAL_TIM_ActiveChannel active_ch;
        switch (dev->TIM_Channel) {
            case TIM_CHANNEL_1: active_ch = HAL_TIM_ACTIVE_CHANNEL_1; break;
            case TIM_CHANNEL_2: active_ch = HAL_TIM_ACTIVE_CHANNEL_2; break;
            case TIM_CHANNEL_3: active_ch = HAL_TIM_ACTIVE_CHANNEL_3; break;
            case TIM_CHANNEL_4: active_ch = HAL_TIM_ACTIVE_CHANNEL_4; break;
            default:            active_ch = HAL_TIM_ACTIVE_CHANNEL_CLEARED; break;
        }

        // 3. Kiểm tra xem ngắt này có đúng là của Kênh (Channel) mà cảm biến đang đăng ký không
        if (htim->Channel == active_ch)
        {
            switch (dev->state)
            {
                case HCSR04_WAIT_RISING_STATE:
                    dev->t_rising = HAL_TIM_ReadCapturedValue(htim, dev->TIM_Channel);
                    dev->state = HCSR04_WAIT_FALLING_STATE;
                    __HAL_TIM_SET_CAPTUREPOLARITY(htim, dev->TIM_Channel, TIM_INPUTCHANNELPOLARITY_FALLING);
                    break;

                case HCSR04_WAIT_FALLING_STATE:
                    dev->t_falling = HAL_TIM_ReadCapturedValue(htim, dev->TIM_Channel);
                    dev->state = HCSR04_COMPLETE_STATE;
                    __HAL_TIM_SET_CAPTUREPOLARITY(htim, dev->TIM_Channel, TIM_INPUTCHANNELPOLARITY_RISING);
                    break;

                default:
                    /* Capture đến trong lúc đã TIMEOUT/COMPLETE/IDLE -> bỏ qua,
                     * tránh ghi đè dữ liệu của chu kỳ đo kế tiếp. */
                    break;
            }
        }
    }
}

void HCSR04_Cliff_Handle(HCSR04_Cliff_t* dev)
{
    if (dev->state == HCSR04_COMPLETE_STATE)
    {
        uint32_t delta_t;
        if (dev->t_falling >= dev->t_rising) {
            delta_t = dev->t_falling - dev->t_rising;
        } else {
            delta_t = (dev->htim->Instance->ARR - dev->t_rising) + dev->t_falling;
        }

        dev->distance = 0.017f * (float)delta_t;
        dev->state = HCSR04_IDLE_STATE;
    }
    else if (dev->state == HCSR04_TIMEOUT_STATE)
    {
        /* Không nhận được echo trong thời gian timeout -> không có vật cản
         * trong tầm đo. Với ứng dụng cliff-detect, đây là trạng hợp "mất an
         * toàn" giống như distance vượt limit, nên ép distance = limit để
         * dùng chung logic lọc nhiễu phía dưới. */
        dev->distance = dev->limit_distance;
        dev->state = HCSR04_IDLE_STATE;
    }
    else
    {
        /* Vẫn đang giữa chu kỳ đo (WAIT_RISING/WAIT_FALLING), chưa có gì
         * mới để xử lý trong lần gọi Handle() này. */
        return;
    }

    if (dev->distance >= dev->limit_distance || dev->distance <= 0.0f)
    {
        if (dev->filter_count < 255U) {
            dev->filter_count++;
        }
        if (dev->filter_count >= 2)
        {
            if (dev->Cliff_Callback != NULL) {
                dev->Cliff_Callback(dev);
            }
        }
    }
    else
    {
        dev->filter_count = 0; /* An toàn -> reset bộ đếm lọc nhiễu */
    }
}

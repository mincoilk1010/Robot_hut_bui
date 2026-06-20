#include "Servo.h"

static TIM_HandleTypeDef *_htim;
static uint32_t _channel;

void Servo_Init(TIM_HandleTypeDef *htim, uint32_t channel) {
    _htim = htim;
    _channel = channel;
    HAL_TIM_PWM_Start(_htim, _channel);
}

void Servo_WriteAngle(uint16_t angle) {
    if (angle > 180) {
        angle = 180;
    }
    // uint16_t CCCR_Value = (1000 + (angle * 1000 / 180 ));
    uint16_t CCCR_Value = (500 + (angle * 2000 / 180 ));
    __HAL_TIM_SET_COMPARE(_htim, _channel, CCCR_Value);
}
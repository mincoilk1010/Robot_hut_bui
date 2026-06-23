#ifndef HCSR04_H
#define HCSR04_H

#include "stm32f1xx_hal.h"

/* Thời gian tối đa chờ 1 lần đo hoàn tất (ms).
 * Nếu quá thời gian này mà không nhận được cạnh ECHO (do mất kết nối,
 * sai chân, hoặc không có vật cản trong tầm đo) -> tự coi là TIMEOUT
 * và quay về trạng thái IDLE, KHÔNG để state machine bị kẹt vĩnh viễn. */
#define HCSR04_TIMEOUT_MS   30U

typedef enum {
    HCSR04_IDLE_STATE,
    HCSR04_WAIT_RISING_STATE,
    HCSR04_WAIT_FALLING_STATE,   /* đã sửa lỗi gõ "WAIR_FALLING" */
    HCSR04_COMPLETE_STATE,
    HCSR04_TIMEOUT_STATE         /* trạng thái mới: không nhận được echo kịp thời */
} HCSR04_State;

typedef struct HCSR04_Cliff {
    GPIO_TypeDef* TRIG_Port;
    uint16_t      TRIG_Pin;
    TIM_HandleTypeDef* htim;
    uint32_t      TIM_Channel;

    uint32_t      t_rising;
    uint32_t      t_falling;
    volatile HCSR04_State state;

    uint32_t      start_tick;    /* tick lúc bắt đầu 1 lần đo, dùng để tính timeout */

    float         distance;
    float         limit_distance;
    uint8_t       filter_count;

    void (*Cliff_Callback)(struct HCSR04_Cliff* dev);
} HCSR04_Cliff_t;


void HCSR04_Cliff_Init(HCSR04_Cliff_t* dev, GPIO_TypeDef* trig_port, uint16_t trig_pin,
                       TIM_HandleTypeDef* htim, uint32_t channel, float limit_dist,
                       void (*callback_func)(HCSR04_Cliff_t*));

void HCSR04_Cliff_Start(HCSR04_Cliff_t* dev);
void HCSR04_Cliff_Handle(HCSR04_Cliff_t* dev);
void HCSR04_Cliff_Capture_Callback(HCSR04_Cliff_t* dev, TIM_HandleTypeDef* htim);

#endif /* HCSR04_H */

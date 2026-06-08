/*
 * encoder.h
 *
 *  Created on: Jun 7, 2026
 *      Author: GB Center
 */


#ifndef MYLIB_INC_ENCODER_H_
#define MYLIB_INC_ENCODER_H_
#include "main.h"
#include "types.h"

/*
typedef struct
{
	i32 pulse_left;
	i32 pulse_right;

	i32 total_pulse_left;
	i32 total_pulse_right;

	float distance_left_m;
	float distance_right_m;
	float distance_robot_m;

	float velocity_left_mps;
	float velocity_right_mps;
	float velocity_robot_mps;
}Encoder_data_t;

extern Encoder_data_t encoder_data;


void Encoder_init();
void Encoder_reset();
void EncoderCounter_Update();
void EncoderCounter_ResetDistance();
*/


typedef struct
{
	u32 period;
	u32 cap_last;
	u32 tick;
	i32 total;
	f32 vel,rpm,dist;
	u8 active;
}Encoder_data_t;

extern Encoder_data_t ec_l;
extern Encoder_data_t ec_r;
extern volatile u32 g_ms ;
extern TIM_HandleTypeDef htim3;

extern TIM_HandleTypeDef htim5;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void encoder_init();

#endif /* MYLIB_INC_ENCODER_H_ */

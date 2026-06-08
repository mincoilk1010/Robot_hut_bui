/*
 * pid.c
 *
 *  Created on: Jun 7, 2026
 *      Author: GB Center
 */

#ifndef INC_PID_C_
#define INC_PID_C_




typedef struct
{
	float kp;
	float ki;
	float kd;

	float error;
	float prev_error;
	float integral;
	float output;

	float output_min;
	float output_max;

	float integral_min;
	float integral_max;
}PID_t;
extern PID_t pid_l;
extern PID_t pid_r;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float output_min,
				float output_max, float integral_min, float intrgral_max);

void PID_Reset(PID_t * pid);

float PID_Update(PID_t *pid, float target, float feedback, float dt);




#endif /* INC_PID_C_ */

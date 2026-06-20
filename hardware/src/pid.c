
/*
 * pid.c
 *
 *  Created on: May 20, 2026
 *      Author: GB Center
 */
#include "pid.h"
#include "types.h"


PID_t pid_l;
PID_t pid_r;
void PID_Init(PID_t *pid, float kp ,float ki, float kd, float output_min,
				float output_max, float integral_min, float intrgral_max)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

	pid->error = 0.0f;
	pid->prev_error = 0.0f;
	pid->integral = 0.0f;
	pid->output = 0.0f;

	pid->output_min = output_min;
	pid->output_max = output_max;
	pid->integral_min = integral_min;
	pid->integral_max = intrgral_max;

}

void PID_Reset(PID_t *pid)
{

	pid->error = 0.0f;
	pid->prev_error = 0.0f;
	pid->integral = 0.0f;
	pid->output = 0.0f;

}

float PID_Update(PID_t *pid, float target, float feedback, float dt)
{

	if(pid == 0) return 0.0f;
	if(dt <= 0.0f) return pid->output;

	pid->error = target - feedback;

	pid->integral += pid->error * dt;
	pid->integral = limit(pid->integral, pid->integral_min, pid->integral_max);

	float derivative = (pid->error - pid->prev_error) / dt;
	pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * derivative;

	pid->prev_error = pid->error;


	return limit(pid->output,pid_out_min,pid_out_max);


}




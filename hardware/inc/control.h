/*
 * control.h
 *
 *  Created on: Jun 7, 2026
 *      Author: GB Center
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

//#include "main.h"
#include "types.h"
#include "pid.h"
#include "motor.h"
#include "encoder.h"
#include "math.h"
#include "mpu6050.h"
#include "delay.h"
typedef struct {
    float x;
    float y;
    float theta; /* rad*/
    float v;
    float w; /*rad/s, vận tốc góc*/
}Pose_t;

extern Pose_t pose;
extern _vo float g_sp_v,g_sp_w;
extern i16 p_l ,p_r ;
extern float sr ,sl ;
extern _vo u8  Flag_Target ;
void pid_setup();
void motorcontrol_pid();

void Kinematics_update(float ds_left, float ds_right);
void Kinematics_inverse(float v, float w, float *vL_out, float *vR_out);

void Kinematics_obs_pos(float dist_m, float servo_deg,  float *obs_x, float *obs_y);

void Kinematics_reset(void);

#endif /* INC_CONTROL_H_ */

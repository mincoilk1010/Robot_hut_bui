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

#define PH_KP_DIST   1.2f
#define PH_KP_HEAD   2.5f
#define PH_MAX_V     0.3f
#define PH_DEAD_M    0.03f   /* 3cm: tại đích */
#define PH_DEAD_R    0.10f   /* 6°: thẳng hướng */

typedef struct {
    float x;
    float y;
    float theta; /* rad*/
    float v;
    float w; /*rad/s, vận tốc góc*/
}Pose_t;
typedef enum
{
    PH_OFF = 0,
    PH_HOLD,
    PH_RETURN

}Ph_state_t;

typedef struct
{
    Ph_state_t state;
    f32 x_t, y_t, theta_t;
    f32 dist_err, head_err;
}PH_t;

extern Pose_t pose;
extern _vo float g_sp_v,g_sp_w;
extern i16 p_l ,p_r ;
extern float sr ,sl ;
extern _vo u8  Flag ;
extern float heading_target;
void HeadingHold_Task(void);
void PH_activate();
void PH_deactivate();
void PH_task();
const char* Ph_state_str();
void pid_setup();
void motorcontrol_pid();

void Kinematics_update(float ds_left, float ds_right);
void Kinematics_inverse(float v, float w, float *vL_out, float *vR_out);

void Kinematics_obs_pos(float dist_m, float servo_deg,  float *obs_x, float *obs_y);

void Kinematics_reset(void);

#endif /* INC_CONTROL_H_ */

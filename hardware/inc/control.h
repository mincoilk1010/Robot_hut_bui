/*
 * control.h
 *
 *  Created on: Jun 7, 2026
 *      Author: GB Center
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

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
#define PH_DEAD_M    0.03f
#define PH_DEAD_R    0.10f

typedef struct {
    float x;
    float y;
    float theta;
    float v;
    float w;
} Pose_t;

typedef enum
{
    PH_OFF = 0,
    PH_HOLD,
    PH_RETURN
} Ph_state_t;

typedef struct
{
    Ph_state_t state;
    f32 x_t, y_t, theta_t;
    f32 dist_err, head_err;
} PH_t;

extern Pose_t pose;
extern _vo float g_sp_v, g_sp_w;
extern i16 p_l, p_r;
extern float sr, sl;
extern _vo u8 Flag;
extern _vo float heading_target;

void HeadingHold_SetTarget(float target_deg);
void HeadingHold_SetTrim(float w_trim);
void HeadingHold_Enable(u8 enable);
void HeadingHold_Task(void);

void PH_activate(void);
void PH_deactivate(void);
void PH_task(void);
const char* Ph_state_str(void);

void pid_setup(void);
void motorcontrol_pid(void);
void Control_Task20ms(void);

void Kinematics_update(float ds_left, float ds_right);
void Kinematics_inverse(float v, float w, float *vL_out, float *vR_out);
void Kinematics_obs_pos(float dist_m, float servo_deg, float *obs_x, float *obs_y);
void Kinematics_reset(void);

#define TR_KP         0.065f
#define TR_KD         0.001f
#define TR_W_MAX      2.20f
#define TR_W_MIN      0.16f
#define TR_SLOW_DEG   18.0f
#define TR_DONE_DEG   1.2f

typedef enum
{
    TR_IDLE,
    TR_ACCEL,
    TR_RUN,
    TR_SETTLE,
    TR_STOP,
    TR_DONE,
    TR_TOUT
} TrSt_t;

typedef struct {
    TrSt_t state;
    float  yaw_t;
    float  err;
    float  err_old;
    float  yaw_final;
    float  w_cmd;
    int8_t dir;
    uint32_t t0, t_settle;
    uint8_t done, timeout;
} Turn_t;

extern Turn_t turn;

void  turn_start(float delta_deg);
void  turn_start_to(float target_deg);
void  turn_task(void);
u8    turn_done(void);
float turn_final_yaw(void);
float turn_residual(void);
float angle_diff(float t, float c);

#endif /* INC_CONTROL_H_ */

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
#define PH_KP_HEAD   2.0f
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
extern f32 prev_dist_l, prev_dist_r ;
extern _vo float heading_target;

/* Heading-hold gains. Error/rate are in deg and deg/s; output is rad/s. */
#define HH_KP_BASE          0.022f
#define HH_KP_SPEED         0.01f
#define HH_KI               0.0050f
#define HH_KD               0.0040f
#define HH_I_MAX           30.0f
#define HH_GYRO_ALPHA       0.20f
#define HH_DEADBAND_DEG     0.15f
#define HH_RATE_DEADBAND    0.50f
#define HH_W_MAX            0.50f
#define HH_W_ACCEL          2.0f
/* Set to -1.0f only if positive g_sp_w makes the measured yaw decrease. */
#define YAW_CMD_SIGN_DEFAULT 1.0f

void HeadingHold_SetTarget(float target_deg);
void HeadingHold_SetTrim(float w_trim);
void HeadingHold_Enable(u8 enable);
void HeadingHold_Task(void);
void PH_activate();
void PH_deactivate();
void PH_task();

void pid_setup();
void motorcontrol_pid();
void Control_Task20ms(void);

void Kinematics_update(float ds_left, float ds_right);
void Kinematics_inverse(float v, float w, float *vL_out, float *vR_out);

void Kinematics_obs_pos(float dist_m, float servo_deg,  float *obs_x, float *obs_y);

void Kinematics_reset(void);



#define TR_KP         0.0384f
#define TR_KD         0.0015f
#define TR_W_MAX      1.45f
#define TR_W_MIN      0.18f
#define TR_SLOW_DEG  25.0f
#define TR_DONE_DEG   3.0f
#define TR_ACCEL_MS   200u
#define TR_SETTLE_MS  120u
#define TR_STOP_MS     80u
#define TURN_TIMEOUT_MS 5000u
typedef enum
{
    TR_IDLE,
    TR_ACCEL,
    TR_RUN,
    TR_SETTLE,
    TR_STOP,
    TR_DONE,
    TR_TOUT
}TrSt_t;
 
typedef struct {
    TrSt_t state;
    float  yaw_t;      
    float  err;       
    float  err_old;
    float  yaw_final;   
    float  w_cmd;      
    float  yaw_start;
    float  delta_cmd;
    int8_t dir;         
    uint32_t t0, t_settle;
    uint8_t  done, timeout;
} Turn_t;
extern volatile Turn_t turn;
void  turn_start(float delta_deg);
void  turn_start_to(float target_deg);
void  turn_task(void);
u8    turn_done(void);
float turn_final_yaw(void);
float turn_residual(void);
float angle_diff(float t,float c);
#endif /* INC_CONTROL_H_ */

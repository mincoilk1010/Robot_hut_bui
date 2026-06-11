/*
 * control.c
 *
 *  Created on: Jun 7, 2026
 *      Author: GB Center
 */



#include "control.h"
#include "mpu6050.h"
#include "math.h"

Pose_t pose ;
PH_t ph = {0};

void HeadingHold_Task(void)
{
    static float err_old = 0.0f;


    float err = mpu6050_angleDiff(heading_target, yaw);


    float derr = (err - err_old) / dt_s;
    err_old = err;


    float speed_factor = 1.0f - fabsf(g_sp_v) / PH_MAX_V;
    if(speed_factor < 0.3f) speed_factor = 0.3f;
    if(fabsf(err) < DEG2RAD(10.0f)) speed_factor *= 0.6f;
    float kp = 0.035f * speed_factor;
    float kd = 0.0025f;


    float kff = 0.02f;
    float ff = kff * g_sp_v;


    g_sp_w =kp * err +kd * derr +ff;


    g_sp_w = limit(g_sp_w, -1.2f, 1.2f);
}

void PH_activate()
{
    ph.x_t  = pose.x;
    ph.y_t = pose.y;
    ph.theta_t = pose.theta;
    ph.state = PH_HOLD;
    g_sp_v = 0.0f;
    g_sp_w = 0.0f;
}

void PH_deactivate()
{
    ph.state = PH_OFF;
    g_sp_v = 0.0f;
    g_sp_w = 0.0f;
}

void PH_task()
{
    if(ph.state == PH_OFF) return;

    f32 dx = ph.x_t - pose.x;
    f32 dy = ph.y_t - pose.y;
    ph.dist_err = sqrtf(dx*dx + dy*dy);

    if(ph.dist_err < PH_DEAD_M)
    {
        ph.state = PH_HOLD;
        g_sp_v = 0.0f;
        f32 th_err = ph.theta_t - pose.theta;
        while(th_err > PI) th_err -= 2.0f*PI;
        while(th_err < -PI) th_err += 2.0f*PI;
        ph.head_err = th_err;
        g_sp_w = (ABS_F(th_err) > PH_DEAD_R) ? limit(PH_KP_HEAD * th_err, -0.5f, 0.5f) : 0.0f;
        return;
    }

    // Đang ở xa chạy về
     ph.state = PH_RETURN;
     f32 ht = atan2f(dy,dx);
     ph.head_err = ht - pose.theta;
     while(ph.head_err > PI) ph.head_err -= 2.0f*PI;
     while(ph.head_err < -PI) ph.head_err += 2.0f*PI;
     if(fabsf(ph.head_err) > DEG2RAD(15.0f))
     {
         g_sp_v = 0.0f;

         g_sp_w =limit(PH_KP_HEAD *ph.head_err,-1.0f, 1.0f);
     }
     else
     {
    	 float v = PH_KP_DIST * ph.dist_err;

    	 if(ph.dist_err < 0.2f) v *= 0.5f;

    	 g_sp_v = limit(v, 0.0f, PH_MAX_V);

    	g_sp_w = limit(0.05f * ph.head_err, -0.5f, 0.5f);
     }
}


void Kinematics_inverse(float v, float w, float *vL_out, float *vR_out)
{

    *vL_out = v - w * (WHEEL_BASE_M * 0.5f);
    *vR_out = v + w * (WHEEL_BASE_M * 0.5f);


}
void Kinematics_update(float ds_left, float ds_right)
{
    float ds = (ds_right + ds_left) * 0.5f;

    float theta_old = pose.theta;

    pose.theta = DEG2RAD(yaw);

    float theta_mid =
            (theta_old + pose.theta) * 0.5f;

    pose.x += ds * cosf(theta_mid);
    pose.y += ds * sinf(theta_mid);

    pose.v = ds / 0.02f;
    float dtheta = pose.theta -theta_old;
    while(dtheta > PI) dtheta -= 2.0f*PI;

    while(dtheta < -PI) dtheta += 2.0f*PI;
    pose.w = dtheta / 0.02f;
}


void Kinematics_obs_pos(float dist_m, float servo_deg,  float *obs_x, float *obs_y)
{
    float alpha = pose.theta + DEG2RAD(servo_deg - 90.0f);
    *obs_x = pose.x + dist_m * cosf(alpha);
    *obs_y = pose.y + dist_m * sinf(alpha);
}


void Kinematics_reset(void)
{
    Pose_t Zero = {0};
    pose = Zero;
}
void pid_setup()
{
    PID_Init(&pid_l,KP_l,KI_l,KD_l,pid_out_min,pid_out_max,pid_int_min,pid_int_max);
    PID_Init(&pid_r,KP_r,KI_r,KD_r,pid_out_min,pid_out_max,pid_int_min,pid_int_max);
}

void motorcontrol_pid()
{

    Kinematics_inverse(g_sp_v,g_sp_w,&sl,&sr);
    /* Left */
    if(fabs(sl) <0.0001f)
    {
        PID_Reset(&pid_l);TIM4->CCR1=0;TIM4->CCR2=0;Motor_Left_Dir=0;
    }
    else{
        i16 p=(i16)PID_Update(&pid_l,sl,ec_l.vel,dt_s);
        p_l = p;

        if(sl > 0.0f && p < 0.0f)
        {
            p=0;
            pid_l.integral = 0.0f;
        }
        if(sl < 0.0f && p > 0.0f)
        {
            p=0;
            pid_l.integral = 0.0f;
        }
        if(p>0){
            Motor_Left_Dir=1;
            TIM4->CCR1=(u32)p;
            TIM4->CCR2=0;
        }
        else if(p<0)
        {
            Motor_Left_Dir=-1;
            TIM4->CCR1=0;
            TIM4->CCR2=(u32)(-p);
        }
        else{
            Motor_Left_Dir=0;
            TIM4->CCR1=0;
            TIM4->CCR2=0;
        }
    }
    /* Right */
    if(fabs(sr) < 0.00001f){
        PID_Reset(&pid_r);
        TIM4->CCR3=0;
        TIM4->CCR4=0;
        Motor_Right_Dir=0;
    }
    else{
        i16 p=(i16)PID_Update(&pid_r,sr,ec_r.vel,dt_s);
        p_r = p;

        if(sr > 0.0f && p < 0.0f)
        {
            p=0;
            pid_r.integral = 0.0f;
        }
        if(sr < 0.0f && p > 0.0f)
        {
            p=0;
            pid_r.integral = 0.0f;
        }
        if(p>0)
        {
            Motor_Right_Dir=1;
            TIM4->CCR3=(u32)p;
            TIM4->CCR4=0;
        }
        else if(p<0)
        {
            Motor_Right_Dir=-1;
            TIM4->CCR3=0;
            TIM4->CCR4=(u32)(-p);
        }
        else
        {
            Motor_Right_Dir=0;
            TIM4->CCR3=0;
            TIM4->CCR4=0;
        }
    }
}


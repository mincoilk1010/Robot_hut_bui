/*
 * control.c
 *
 *  Created on: Jun 7, 2026
 *      Author: GB Center
 */



#include "control.h"



Pose_t pose ;
_vo u8  Flag_Target = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

    if(GPIO_Pin == GPIO_PIN_12)
    {
    	Flag_Target = !Flag_Target;
        static f32 prev_dl=0, prev_dr=0;
        f32 ds_l = ec_l.dist - prev_dl;
        f32 ds_r = ec_r.dist - prev_dr;
        prev_dl  = ec_l.dist;
        prev_dr  = ec_r.dist;
        Kinematics_update(ds_l, ds_r);
        mpu6050_readAccl();
        mpu6050_readGyro();
        mpu6050_filter(0.01f);
        if(Flag_Target == 0)
        {
            if(ec_l.active)
            {

                if((millis() - ec_l.tick) > 80)
                {
                    ec_l.vel = 0;
                    ec_l.rpm = 0;
                    ec_l.active = 0;
                }
            }

            if(ec_r.active)
            {
                if((millis() - ec_r.tick) > 80)
                {
                    ec_r.vel = 0;
                    ec_r.rpm = 0;
                    ec_r.active = 0;
                }
            }
        	motorcontrol_pid();
        }
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
    float dth = (ds_right - ds_left) / WHEEL_BASE_M;

    float th_mid = pose.theta + dth * 0.5f;

    pose.x += ds * cosf(th_mid);
    pose.y += ds * sinf(th_mid);
    pose.theta += dth;
    while(pose.theta > PI) pose.theta -= 2.0f * PI;
    while(pose.theta < -PI) pose.theta += 2.0f * PI;

    pose.v = ds / dt_s;
    pose.w = dth / dt_s;
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
    if(sl==0.0f)
    {
        PID_Reset(&pid_l);TIM4->CCR1=0;TIM4->CCR2=0;Motor_Left_Dir=0;
    }
    else{
        i16 p=(i16)PID_Update(&pid_l,sl,ec_l.vel,dt_ms);
        p_l = p;
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
    if(sr==0.0f){
        PID_Reset(&pid_r);
        TIM4->CCR3=0;
        TIM4->CCR4=0;
        Motor_Right_Dir=0;
    }
    else{
        i16 p=(i16)PID_Update(&pid_r,sr,ec_r.vel,dt_ms);
        p_r = p;
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
/*
void PosHold_deactivate(void)
{
    ph.state = PH_OFF;
    g_sp_v = 0.0f; g_sp_w = 0.0f;
    PID_Reset(&pid_l); PID_Reset(&pid_r);
    motor_stop();
}

void PosHold_task(void)
{
    if (ph.state == PH_OFF) return;


    float dx = ph.x_t - pose.x;
    float dy = ph.y_t - pose.y;
    ph.dist_err = sqrtf(dx*dx + dy*dy);

    if (ph.dist_err < PH_DEAD_M) {

        ph.state = PH_HOLD;
        g_sp_v   = 0.0f;


        float th_err = ph.theta_t - pose.theta;
        while (th_err >  PI) th_err -= 2.0f*PI;
        while (th_err < -PI) th_err += 2.0f*PI;
        g_sp_w = (fabsf(th_err) > PH_HEAD_RAD)
                 ? limit(PH_KP_HEAD * th_err, -0.5f, 0.5f)
                 : 0.0f;
        ph.head_err = th_err;
        return;
    }


    ph.state = PH_RETURN;


    float head_target = atan2f(dy, dx);
    ph.head_err = head_target - pose.theta;
    while (ph.head_err >  PI) ph.head_err -= 2.0f*PI;
    while (ph.head_err < -PI) ph.head_err += 2.0f*PI;


    float v = PH_KP_DIST * ph.dist_err * cosf(ph.head_err);
    v = limit(v, -PH_MAX_V, PH_MAX_V);
    float w = PH_KP_HEAD * ph.head_err;
    w = limit(w, -0.8f, 0.8f);

    g_sp_v = v;
    g_sp_w = w;
}
*/

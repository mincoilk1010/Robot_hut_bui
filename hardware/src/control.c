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
    float kp = 0.04f * speed_factor;
    float kd = 0.0025f;


    float kff = 0.01f;
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
    if(fabs(sr) < 0.0001f){
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


Turn_t turn = {0};


/*
void turn_start(float delta_deg)
{

    while (delta_deg >  180.0f) delta_deg -= 360.0f;
    while (delta_deg <= -180.0f) delta_deg += 360.0f;

    turn.yaw_t    = yaw + delta_deg;
    while(turn.yaw_t > 180.0f) turn.yaw_t -= 360.0f;

    while(turn.yaw_t <= -180.0f)turn.yaw_t += 360.0f;
    turn.err      = delta_deg;
    turn.err_old  = delta_deg;
    turn.dir      = (delta_deg >= 0.0f) ? 1 : -1;
    turn.done     = 0;
    turn.timeout  = 0;
    turn.t0       = HAL_GetTick();
    turn.t_settle = 0;
    turn.state    = TR_RUN;

    g_sp_v = 0.0f;
}

void turn_task(void)
{
    if (turn.state == TR_IDLE || turn.state == TR_DONE || turn.state == TR_TOUT)
        return;

    if ((u32)(HAL_GetTick() - turn.t0) > TURN_TIMEOUT_MS)
    {
        g_sp_v = 0.0f; g_sp_w = 0.0f;
        turn.state   = TR_TOUT;
        turn.done    = 1;
        turn.timeout = 1;
        return;
    }

    float err = turn.yaw_t - yaw;
    while (err >  180.0f) err -= 360.0f;
    while (err <= -180.0f) err += 360.0f;
    turn.err = err;
    float d_err = (err - turn.err_old) / dt_s;
    turn.err_old = err;
    float w = TR_KP * err + TR_KD * d_err;

    float w_abs = ABS_F(w);
    if (w_abs < TR_W_MIN && w_abs > 0.001f)
        w = (w > 0.0f) ? TR_W_MIN : -TR_W_MIN;
    if (w_abs > TR_W_MAX)
        w = (w > 0.0f) ? TR_W_MAX : -TR_W_MAX;
    turn.w_cmd = w;

    if (ABS_F(err) < TR_DONE_DEG)
    {
        if (turn.state != TR_SETTLE)
        {
            turn.state    = TR_SETTLE;
            turn.t_settle = HAL_GetTick();
        }

        g_sp_w = 0.0f;

        if ((u32)(HAL_GetTick() - turn.t_settle) >= TURN_SETTLE_MS)
        {
            turn.yaw_final = yaw;
            g_sp_v = 0.0f; g_sp_w = 0.0f;
            turn.state = TR_DONE;
            turn.done  = 1;
        }
        return;
    }
    turn.state = TR_RUN;
    g_sp_v = 0.0f;
    g_sp_w = w;
}
 
u8 turn_done(void)
{
    return turn.done;
}

float turn_final_yaw(void)
{
    return turn.yaw_final;
}

float turn_residual(void)
{
    float r = turn.yaw_t - turn.yaw_final;
    while (r >  180.0f) r -= 360.0f;
    while (r <= -180.0f) r += 360.0f;
    return r;
}
    */
     
static float _norm(float a)
{ while(a>180.0f)a-=360.0f; while(a<=-180.0f)a+=360.0f; return a; }

/* Relationship between positive chassis w and the installed MPU yaw sign.
 * It is learned on the first turn, so mounting the MPU with Z inverted does
 * not turn the yaw loop into positive feedback. */
static float _turn_yaw_sign = 0.0f;
 
void turn_start(float delta_deg)
{
    delta_deg = _norm(delta_deg);

    turn.yaw_start = yaw;
    turn.delta_cmd = delta_deg;
    turn.yaw_t    = _norm(yaw + delta_deg *
                          ((_turn_yaw_sign == 0.0f) ? 1.0f : _turn_yaw_sign));
    turn.err      = _norm(turn.yaw_t - yaw);
    turn.err_old  = turn.err;
    turn.dir      = (delta_deg >= 0.0f) ? 1 : -1;
    turn.w_cmd    = 0.0f;       /* bắt đầu từ 0 — ACCEL sẽ ramp lên */
    turn.done     = 0;
    turn.timeout  = 0;
    turn.t0       = HAL_GetTick();
    turn.t_settle = 0;
    turn.state    = TR_ACCEL;  /* ★ bắt đầu bằng tăng tốc êm, không nhảy thẳng RUN */
 
    g_sp_v = 0.0f;
    g_sp_w = 0.0f;
}
 
/* ════════════════════════════════════════════════════════════════
 * turn_task() — 5 state đầy đủ, chạy mỗi 10ms (100Hz)
 *
 *   ACCEL  : ramp ω từ 0 lên hướng quay trong TR_ACCEL_MS (hoặc
 *            tới khi |err| < TR_SLOW_DEG) — tránh giật/trượt bánh
 *            lúc bắt đầu, giúp encoder bám đúng góc thật hơn.
 *
 *   RUN    : PD controller. ★ decel zone: khi |err| < TR_SLOW_DEG,
 *            scale ω xuống tuyến tính theo |err|/TR_SLOW_DEG (đây
 *            là TR_SLOW_DEG mà bản cũ khai báo nhưng chưa hề dùng).
 *
 *   SETTLE : |err| < TR_DONE_DEG → cắt ω=0, giữ đủ TR_SETTLE_MS
 *            liên tục (nếu lệch ra lại → quay về RUN).
 *
 *   STOP   : ω đã = 0 từ SETTLE, đợi thêm TR_STOP_MS để bánh thật
 *            sự dừng hẳn (hết trớn cơ khí) rồi mới chốt yaw_final
 *            → góc đọc được CHÍNH XÁC góc thật xe đang đứng, không
 *            phải góc lúc còn đang trôi theo đà.
 *
 *   DONE   : hoàn tất, turn_residual() cho biết sai số còn lại.
 * ════════════════════════════════════════════════════════════════ */
void turn_task(void)    
{
    if (turn.state==TR_IDLE || turn.state==TR_DONE || turn.state==TR_TOUT)
        return;
 
    u32 now = HAL_GetTick();
    if ((u32)(now - turn.t0) > TURN_TIMEOUT_MS) {
        g_sp_v=0; g_sp_w=0;
        turn.state=TR_TOUT; turn.done=1; turn.timeout=1;
        return;
    }
 
    float err = _norm(turn.yaw_t - yaw);
    float d_err = _norm(err - turn.err_old) / dt_s;
    turn.err_old = err;
    turn.err = err;
 
    switch (turn.state) {
 
    case TR_ACCEL: {
        float target = (float)turn.dir * TR_W_MAX;
        /* Bước tăng mỗi chu kỳ để đạt W_MAX trong TR_ACCEL_MS */
        float step = TR_W_MAX * dt_s / (TR_ACCEL_MS / 1000.0f);
        float dw = target - turn.w_cmd;
        if (dw >  step) dw =  step;
        if (dw < -step) dw = -step;
        turn.w_cmd += dw;
        g_sp_v = 0.0f;
        g_sp_w = turn.w_cmd;

        if (_turn_yaw_sign == 0.0f) {
            float moved = _norm(yaw - turn.yaw_start);
            if (ABS_F(moved) >= 1.0f) {
                _turn_yaw_sign = (moved * (float)turn.dir >= 0.0f) ? 1.0f : -1.0f;
                turn.yaw_t = _norm(turn.yaw_start + turn.delta_cmd * _turn_yaw_sign);
                turn.err = _norm(turn.yaw_t - yaw);
                turn.err_old = turn.err;
            }
        }

        if (_turn_yaw_sign != 0.0f &&
            ((now - turn.t0) >= TR_ACCEL_MS || ABS_F(turn.err) < TR_SLOW_DEG))
            turn.state = TR_RUN;
        break;
    }
 
    case TR_RUN: {
        float w = _turn_yaw_sign * (TR_KP*err + TR_KD*d_err);
 
        /* ★ Decel zone: scale êm khi gần đích — đây là phần code cũ
         *   khai báo TR_SLOW_DEG nhưng KHÔNG hề dùng tới */
        if (ABS_F(err) < TR_SLOW_DEG) {
            float scale = ABS_F(err) / TR_SLOW_DEG;
            float floor_scale = TR_W_MIN / TR_W_MAX;
            if (scale < floor_scale) scale = floor_scale;
            w *= scale;
        }
 
        float w_abs = ABS_F(w);
        if (w_abs > TR_W_MAX) w = (w>0)?TR_W_MAX:-TR_W_MAX;
        if (w_abs < TR_W_MIN && w_abs > 0.001f) w = (w>0)?TR_W_MIN:-TR_W_MIN;
 
        turn.w_cmd = w;
        g_sp_v = 0.0f;
        g_sp_w = w;
 
        if (ABS_F(err) < TR_DONE_DEG) {
            turn.t_settle = now;
            turn.state = TR_SETTLE;
        }
        break;
    }
 
    case TR_SETTLE:
        g_sp_v = 0.0f;
        g_sp_w = 0.0f;
        turn.w_cmd = 0.0f;
 
        /* Lệch ra khỏi ngưỡng trong lúc settle → quay lại RUN */
        if (ABS_F(err) > TR_DONE_DEG * 2.0f) {
            turn.state = TR_RUN;
            break;
        }
        if ((now - turn.t_settle) >= TR_SETTLE_MS) {
            turn.t_settle = now;   /* tái dùng làm mốc thời gian cho STOP */
            turn.state = TR_STOP;
        }
        break;
 
    case TR_STOP:
        g_sp_v = 0.0f;
        g_sp_w = 0.0f;
        if ((now - turn.t_settle) >= TR_STOP_MS) {
            turn.yaw_final = yaw;   /* ★ chốt SAU khi đã dừng hẳn */
            g_sp_v = 0.0f; g_sp_w = 0.0f;
            turn.state = TR_DONE;
            turn.done  = 1;
        }
        break;
 
    default: break;
    }
}
 
u8    turn_done(void)      { return turn.done; }
float turn_final_yaw(void) { return turn.yaw_final; }
float turn_residual(void)  {
    float r = _norm(turn.yaw_t - turn.yaw_final);
    return (_turn_yaw_sign == 0.0f) ? r : r * _turn_yaw_sign;
}
float angle_diff(float t,float c)
{
	float d=t-c;
	if(d>180)d-=360;
	if(d<-180)d+=360;
	return d;}

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

static float _turn_yaw_sign = 0.0f;
static float heading_integral = 0.0f;
static float heading_gyro_filt = 0.0f;
static float heading_w_cmd = 0.0f;
static volatile float heading_w_trim = 0.0f;
static volatile u8 heading_hold_enabled = 1u;
static u8 _turn_absolute = 0u;

void HeadingHold_SetTarget(float target_deg)
{
    while(target_deg > 180.0f) target_deg -= 360.0f;
    while(target_deg <= -180.0f) target_deg += 360.0f;

    heading_target = target_deg;
    /* A new heading is a new control manoeuvre: do not retain old wind-up. */
    heading_integral = 0.0f;
    heading_gyro_filt = GZ - GZ_calib;
    heading_w_cmd = 0.0f;
    heading_w_trim = 0.0f;
}

void HeadingHold_SetTrim(float w_trim)
{
    heading_w_trim = limit(w_trim, -HH_W_MAX, HH_W_MAX);
}

void HeadingHold_Enable(u8 enable)
{
    heading_hold_enabled = enable ? 1u : 0u;
    if (!heading_hold_enabled) {
        heading_integral = 0.0f;
        heading_w_cmd = 0.0f;
        heading_w_trim = 0.0f;
        g_sp_w = 0.0f;
    }
}

void HeadingHold_Task(void)
{
    if (!heading_hold_enabled) {
        g_sp_w = 0.0f;
        return;
    }

    float err = mpu6050_angleDiff(heading_target, yaw);
    float gyro_z = GZ - GZ_calib;
    heading_gyro_filt += HH_GYRO_ALPHA * (gyro_z - heading_gyro_filt);

    /* More forward speed needs slightly stronger, not weaker, correction. */
    float speed_ratio = fabsf(g_sp_v) / PH_MAX_V;
    speed_ratio = limit(speed_ratio, 0.0f, 1.0f);
    float kp = HH_KP_BASE + HH_KP_SPEED * speed_ratio;

    float err_ctrl = err;
    if (fabsf(err_ctrl) < HH_DEADBAND_DEG) {
        err_ctrl = 0.0f;
        heading_integral *= 0.90f;
    } else {
        heading_integral += err_ctrl * dt_s;
        heading_integral = limit(heading_integral, -HH_I_MAX, HH_I_MAX);
    }

    /* Gyro rate is a cleaner damping signal than differentiating noisy yaw. */
    float w_sensor = kp * err_ctrl
                   + HH_KI * heading_integral
                   - HH_KD * heading_gyro_filt
                   + heading_w_trim;
    float yaw_sign = (_turn_yaw_sign == 0.0f) ? YAW_CMD_SIGN_DEFAULT
                                               : _turn_yaw_sign;
    float w_target = limit(yaw_sign * w_sensor, -HH_W_MAX, HH_W_MAX);

    if (fabsf(err) < HH_DEADBAND_DEG &&
        fabsf(heading_gyro_filt) < HH_RATE_DEADBAND &&
        fabsf(heading_w_trim) < 0.001f) {
        w_target = 0.0f;
        heading_integral = 0.0f;
    }

    /* Slew limiting prevents alternating wheel commands on successive ticks. */
    float max_step = HH_W_ACCEL * dt_s;
    float dw = w_target - heading_w_cmd;
    dw = limit(dw, -max_step, max_step);
    heading_w_cmd += dw;

    if (w_target == 0.0f && fabsf(heading_w_cmd) < max_step)
        heading_w_cmd = 0.0f;

    g_sp_w = heading_w_cmd;
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

void Control_Task20ms(void)
{
    static volatile u32 last_ms = 0u;
    u32 now = HAL_GetTick();

    /* This function can be requested by both main and the MPU EXTI. Claim
     * one 20 ms slot atomically so heading/turn and wheel PID never run twice. */
    u32 primask = __get_PRIMASK();
    __disable_irq();
    if ((u32)(now - last_ms) < dt_ms) {
        if (!primask) __enable_irq();
        return;
    }
    last_ms = now;
    if (!primask) __enable_irq();

    if (turn.state == TR_IDLE || turn.state == TR_DONE ||
        turn.state == TR_TOUT) {
        HeadingHold_Task();
    } else {
        turn_task();
    }

    motorcontrol_pid();
}

static float motor_pwm_l = 0.0f;
static float motor_pwm_r = 0.0f;

static i16 wheel_control(PID_t *pid, float target, float feedback,
                         float feedforward, float pwm_min,
                         float *pwm_applied)
{
    if (fabsf(target) < MOTOR_SP_DEADBAND) {
        PID_Reset(pid);
        *pwm_applied = 0.0f;
        return 0;
    }

    float dir = (target > 0.0f) ? 1.0f : -1.0f;
    float desired = PID_Update(pid, target, feedback, dt_s)
                  + dir * feedforward;

    /* Do not command reverse braking when the requested wheel direction has
     * not changed. Coast briefly instead; it avoids alternating H-bridge
     * direction at low encoder resolution. */
    if (desired * dir <= 0.0f) {
        desired = 0.0f;
        pid->integral *= 0.8f;
    } else if (fabsf(desired) < pwm_min) {
        desired = dir * pwm_min;
    }
    desired = limit(desired, pid_out_min, pid_out_max);

    /* On a direction reversal, ramp to zero first, then ramp into the new
     * direction. This is gentler on the gearbox and motor driver. */
    if ((*pwm_applied) * dir < 0.0f)
        desired = 0.0f;

    float dp = desired - *pwm_applied;
    dp = limit(dp, -MOTOR_PWM_SLEW_STEP, MOTOR_PWM_SLEW_STEP);
    *pwm_applied += dp;

    if (fabsf(*pwm_applied) < 0.5f) *pwm_applied = 0.0f;
    return (i16)(*pwm_applied);
}

static void apply_left_pwm(i16 pwm)
{
    p_l = pwm;
    if (pwm > 0) {
        Motor_Left_Dir = 1;
        TIM4->CCR1 = (u32)pwm;
        TIM4->CCR2 = 0u;
    } else if (pwm < 0) {
        Motor_Left_Dir = -1;
        TIM4->CCR1 = 0u;
        TIM4->CCR2 = (u32)(-pwm);
    } else {
        Motor_Left_Dir = 0;
        TIM4->CCR1 = 0u;
        TIM4->CCR2 = 0u;
    }
}

static void apply_right_pwm(i16 pwm)
{
    p_r = pwm;
    if (pwm > 0) {
        Motor_Right_Dir = 1;
        TIM4->CCR3 = (u32)pwm;
        TIM4->CCR4 = 0u;
    } else if (pwm < 0) {
        Motor_Right_Dir = -1;
        TIM4->CCR3 = 0u;
        TIM4->CCR4 = (u32)(-pwm);
    } else {
        Motor_Right_Dir = 0;
        TIM4->CCR3 = 0u;
        TIM4->CCR4 = 0u;
    }
}

void motorcontrol_pid(void)
{
    Kinematics_inverse(g_sp_v, g_sp_w, &sl, &sr);

    i16 left_pwm = wheel_control(&pid_l, sl, ec_l.vel,
                                 MOTOR_PWM_FF_L, MOTOR_PWM_MIN_L,
                                 &motor_pwm_l);
    i16 right_pwm = wheel_control(&pid_r, sr, ec_r.vel,
                                  MOTOR_PWM_FF_R, MOTOR_PWM_MIN_R,
                                  &motor_pwm_r);

    apply_left_pwm(left_pwm);
    apply_right_pwm(right_pwm);
}


volatile Turn_t turn = {0};



     
static float _norm(float a)
{ while(a>180.0f)a-=360.0f; while(a<=-180.0f)a+=360.0f; return a; }


void turn_start(float delta_deg)
{
    _turn_absolute = 0u;
    delta_deg = _norm(delta_deg);

    turn.yaw_start = yaw;
    turn.delta_cmd = delta_deg;
    turn.yaw_t    = _norm(yaw + delta_deg *
                          ((_turn_yaw_sign == 0.0f) ? YAW_CMD_SIGN_DEFAULT
                                                    : _turn_yaw_sign));
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

void turn_start_to(float target_deg)
{
    float target = _norm(target_deg);
    float yaw_sign = (_turn_yaw_sign == 0.0f) ? YAW_CMD_SIGN_DEFAULT
                                               : _turn_yaw_sign;
    float sensor_delta = _norm(target - yaw);

    turn_start(sensor_delta / yaw_sign);
    _turn_absolute = 1u;
    turn.yaw_t = target;
    turn.err = _norm(turn.yaw_t - yaw);
    turn.err_old = turn.err;
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
                if (!_turn_absolute)
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

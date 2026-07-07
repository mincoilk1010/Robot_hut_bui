/*
 * control.c
 *
 *  Created on: Jun 7, 2026
 *      Author: GB Center
 */

#include "control.h"
#include "mpu6050.h"
#include "math.h"

Pose_t pose;
PH_t ph = {0};
Turn_t turn = {0};

static float heading_err_old = 0.0f;
static float heading_err_i = 0.0f;
static volatile u8 heading_hold_enabled = 1u;
static volatile float heading_w_trim = 0.0f;

#define TURN_TIMEOUT_MS     5500u
#define TURN_SETTLE_MS      120u
#define HH_DEADBAND_DEG     0.30f
#define HH_KP_MIN           0.026f
#define HH_KP_SPEED         0.016f
#define HH_KI_STRAIGHT      0.0008f
#define HH_KD_STRAIGHT      0.0020f
#define HH_W_LIMIT          0.38f
#define HH_I_LIMIT_DEG_S    25.0f
#define HH_MOVING_TARGET_UPDATE_DEG 8.0f

static float norm_deg(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

float angle_diff(float t, float c)
{
    return norm_deg(t - c);
}

static u8 heading_straight_move_active(void)
{
    return (fabsf(g_sp_v) >= MOTOR_SP_DEADBAND &&
            (turn.state == TR_IDLE || turn.state == TR_DONE ||
             turn.state == TR_TOUT)) ? 1u : 0u;
}

void HeadingHold_SetTarget(float target_deg)
{
    float target = norm_deg(target_deg);

    /* Neu xe dang chay thang, khong cho target troi theo yaw hien tai.
     * Truong hop hay gap: vong while goi HeadingHold_SetTarget(yaw) lien tuc,
     * robot lech den dau thi target cung doi den do -> mat giu huong.
     * Van cho phep doi target lon (vi du sau khi xoay 90 do).
     */
    if (heading_hold_enabled && heading_straight_move_active() &&
        fabsf(angle_diff(target, heading_target)) < HH_MOVING_TARGET_UPDATE_DEG) {
        return;
    }

    heading_target = target;
    heading_err_old = 0.0f;
    heading_err_i = 0.0f;
}

void HeadingHold_SetTrim(float w_trim)
{
    heading_w_trim = limit(w_trim, -0.5f, 0.5f);
}

void HeadingHold_Enable(u8 enable)
{
    heading_hold_enabled = enable ? 1u : 0u;
    if (!heading_hold_enabled) {
        heading_err_old = 0.0f;
        heading_err_i = 0.0f;
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
    /*
     * Giu an toan cho project hien tai: khi khong co lenh tien va khong
     * dang turn_start(), heading hold khong tu xoay xe luc idle/debug.
     */
    if (fabsf(g_sp_v) < MOTOR_SP_DEADBAND &&
        (turn.state == TR_IDLE || turn.state == TR_DONE ||
         turn.state == TR_TOUT)) {
        heading_err_old = 0.0f;
        heading_err_i = 0.0f;
        g_sp_w = 0.0f;
        return;
    }

    float err = mpu6050_angleDiff(heading_target, yaw);
    float derr = angle_diff(err, heading_err_old) / dt_s;
    heading_err_old = err;

    if (fabsf(err) < HH_DEADBAND_DEG) {
        err = 0.0f;
        derr = 0.0f;
        heading_err_i *= 0.92f;
    } else {
        heading_err_i += err * dt_s;
        heading_err_i = limit(heading_err_i,
                              -HH_I_LIMIT_DEG_S,
                              HH_I_LIMIT_DEG_S);
    }

    float speed_ratio = fabsf(g_sp_v) / PH_MAX_V;
    speed_ratio = limit(speed_ratio, 0.0f, 1.0f);

    float kp = HH_KP_MIN + HH_KP_SPEED * speed_ratio;
    float ki = HH_KI_STRAIGHT;
    float kd = HH_KD_STRAIGHT;

    g_sp_w = kp * err + ki * heading_err_i + kd * derr + heading_w_trim;
    g_sp_w = limit(g_sp_w, -HH_W_LIMIT, HH_W_LIMIT);
}

void PH_activate(void)
{
    ph.x_t = pose.x;
    ph.y_t = pose.y;
    ph.theta_t = pose.theta;
    ph.state = PH_HOLD;
    g_sp_v = 0.0f;
    g_sp_w = 0.0f;
}

void PH_deactivate(void)
{
    ph.state = PH_OFF;
    g_sp_v = 0.0f;
    g_sp_w = 0.0f;
}

void PH_task(void)
{
    if (ph.state == PH_OFF) return;

    f32 dx = ph.x_t - pose.x;
    f32 dy = ph.y_t - pose.y;
    ph.dist_err = sqrtf(dx * dx + dy * dy);

    if (ph.dist_err < PH_DEAD_M)
    {
        ph.state = PH_HOLD;
        g_sp_v = 0.0f;

        f32 th_err = ph.theta_t - pose.theta;
        while (th_err > PI) th_err -= 2.0f * PI;
        while (th_err < -PI) th_err += 2.0f * PI;

        ph.head_err = th_err;
        g_sp_w = (ABS_F(th_err) > PH_DEAD_R)
               ? limit(PH_KP_HEAD * th_err, -0.5f, 0.5f)
               : 0.0f;
        return;
    }

    ph.state = PH_RETURN;
    f32 ht = atan2f(dy, dx);
    ph.head_err = ht - pose.theta;
    while (ph.head_err > PI) ph.head_err -= 2.0f * PI;
    while (ph.head_err < -PI) ph.head_err += 2.0f * PI;

    if (fabsf(ph.head_err) > DEG2RAD(15.0f))
    {
        g_sp_v = 0.0f;
        g_sp_w = limit(PH_KP_HEAD * ph.head_err, -1.0f, 1.0f);
    }
    else
    {
        float v = PH_KP_DIST * ph.dist_err;
        if (ph.dist_err < 0.2f) v *= 0.5f;

        g_sp_v = limit(v, 0.0f, PH_MAX_V);
        g_sp_w = limit(0.05f * ph.head_err, -0.5f, 0.5f);
    }
}

const char* Ph_state_str(void)
{
    switch (ph.state) {
    case PH_OFF:    return "OFF";
    case PH_HOLD:   return "HOLD";
    case PH_RETURN: return "RETURN";
    default:        return "?";
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

    float theta_mid = (theta_old + pose.theta) * 0.5f;

    pose.x += ds * cosf(theta_mid);
    pose.y += ds * sinf(theta_mid);

    pose.v = ds / 0.02f;

    float dtheta = pose.theta - theta_old;
    while (dtheta > PI) dtheta -= 2.0f * PI;
    while (dtheta < -PI) dtheta += 2.0f * PI;
    pose.w = dtheta / 0.02f;
}

void Kinematics_obs_pos(float dist_m, float servo_deg, float *obs_x, float *obs_y)
{
    float alpha = pose.theta + DEG2RAD(servo_deg - 90.0f);
    *obs_x = pose.x + dist_m * cosf(alpha);
    *obs_y = pose.y + dist_m * sinf(alpha);
}

void Kinematics_reset(void)
{
    Pose_t zero = {0};
    pose = zero;
}

void pid_setup(void)
{
    PID_Init(&pid_l, KP_l, KI_l, KD_l,
             pid_out_min, pid_out_max, pid_int_min, pid_int_max);
    PID_Init(&pid_r, KP_r, KI_r, KD_r,
             pid_out_min, pid_out_max, pid_int_min, pid_int_max);
}

static i16 motor_start_boost(float target, float feedback,
                             i16 pwm, float pwm_min)
{
    if (fabsf(target) < MOTOR_SP_DEADBAND)
        return 0;

    float dir = (target > 0.0f) ? 1.0f : -1.0f;
    float fb_along_cmd = feedback * dir;
    float target_abs = fabsf(target);

    if ((float)pwm * dir > 0.0f &&
        fb_along_cmd < (target_abs - 0.015f) &&
        fabsf((float)pwm) < pwm_min) {
        pwm = (i16)(dir * pwm_min);
    }

    return pwm;
}

static i16 motor_pwm_slew(i16 desired, i16 *applied)
{
    if ((*applied > 0 && desired < 0) ||
        (*applied < 0 && desired > 0)) {
        desired = 0;
    }

    i32 diff = (i32)desired - (i32)(*applied);
    if (diff > (i32)MOTOR_PWM_SLEW_STEP)
        diff = (i32)MOTOR_PWM_SLEW_STEP;
    if (diff < -(i32)MOTOR_PWM_SLEW_STEP)
        diff = -(i32)MOTOR_PWM_SLEW_STEP;

    *applied = (i16)((i32)(*applied) + diff);
    if (*applied > -2 && *applied < 2)
        *applied = 0;

    return *applied;
}

void Control_Task20ms(void)
{
    static volatile u32 last_ms = 0u;
    u32 now = HAL_GetTick();

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

void motorcontrol_pid(void)
{
    static i16 pwm_l_applied = 0;
    static i16 pwm_r_applied = 0;

    Kinematics_inverse(g_sp_v, g_sp_w, &sl, &sr);

    if (fabsf(sl) < 0.0001f)
    {
        PID_Reset(&pid_l);
        TIM4->CCR1 = 0;
        TIM4->CCR2 = 0;
        Motor_Left_Dir = 0;
        p_l = 0;
        pwm_l_applied = 0;
    }
    else
    {
        i16 p = (i16)PID_Update(&pid_l, sl, ec_l.vel, dt_s);

        if (sl > 0.0f && p < 0) {
            p = 0;
            pid_l.integral = 0.0f;
        }
        if (sl < 0.0f && p > 0) {
            p = 0;
            pid_l.integral = 0.0f;
        }
        p = motor_start_boost(sl, ec_l.vel, p, MOTOR_PWM_MIN_L);
        p = motor_pwm_slew(p, &pwm_l_applied);
        p_l = p;

        if (p > 0) {
            Motor_Left_Dir = 1;
            TIM4->CCR1 = (u32)p;
            TIM4->CCR2 = 0;
        } else if (p < 0) {
            Motor_Left_Dir = -1;
            TIM4->CCR1 = 0;
            TIM4->CCR2 = (u32)(-p);
        } else {
            Motor_Left_Dir = 0;
            TIM4->CCR1 = 0;
            TIM4->CCR2 = 0;
        }
    }

    if (fabsf(sr) < 0.00001f)
    {
        PID_Reset(&pid_r);
        TIM4->CCR3 = 0;
        TIM4->CCR4 = 0;
        Motor_Right_Dir = 0;
        p_r = 0;
        pwm_r_applied = 0;
    }
    else
    {
        i16 p = (i16)PID_Update(&pid_r, sr, ec_r.vel, dt_s);

        if (sr > 0.0f && p < 0) {
            p = 0;
            pid_r.integral = 0.0f;
        }
        if (sr < 0.0f && p > 0) {
            p = 0;
            pid_r.integral = 0.0f;
        }
        p = motor_start_boost(sr, ec_r.vel, p, MOTOR_PWM_MIN_R);
        p = motor_pwm_slew(p, &pwm_r_applied);
        p_r = p;

        if (p > 0) {
            Motor_Right_Dir = 1;
            TIM4->CCR3 = (u32)p;
            TIM4->CCR4 = 0;
        } else if (p < 0) {
            Motor_Right_Dir = -1;
            TIM4->CCR3 = 0;
            TIM4->CCR4 = (u32)(-p);
        } else {
            Motor_Right_Dir = 0;
            TIM4->CCR3 = 0;
            TIM4->CCR4 = 0;
        }
    }
}

void turn_start(float delta_deg)
{
    delta_deg = norm_deg(delta_deg);

    turn.yaw_t = norm_deg(yaw + delta_deg);
    turn.err = delta_deg;
    turn.err_old = delta_deg;
    turn.dir = (delta_deg >= 0.0f) ? 1 : -1;
    turn.done = 0;
    turn.timeout = 0;
    turn.t0 = HAL_GetTick();
    turn.t_settle = 0;
    turn.state = TR_RUN;

    g_sp_v = 0.0f;
}

void turn_start_to(float target_deg)
{
    turn_start(angle_diff(norm_deg(target_deg), yaw));
}

void turn_task(void)
{
    if (turn.state == TR_IDLE || turn.state == TR_DONE || turn.state == TR_TOUT)
        return;

    if ((u32)(HAL_GetTick() - turn.t0) > TURN_TIMEOUT_MS)
    {
        g_sp_v = 0.0f;
        g_sp_w = 0.0f;
        turn.state = TR_TOUT;
        turn.done = 1;
        turn.timeout = 1;
        return;
    }

    float err = angle_diff(turn.yaw_t, yaw);
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
            turn.state = TR_SETTLE;
            turn.t_settle = HAL_GetTick();
        }

        g_sp_w = 0.0f;

        if ((u32)(HAL_GetTick() - turn.t_settle) >= TURN_SETTLE_MS)
        {
            turn.yaw_final = yaw;
            g_sp_v = 0.0f;
            g_sp_w = 0.0f;
            turn.state = TR_DONE;
            turn.done = 1;
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
    return angle_diff(turn.yaw_t, turn.yaw_final);
}

/*
 * robot_task.c
 */

#include "robot_task.h"
#include "navigation.h"
#include "control.h"
#include "mpu6050.h" // Chứa biến yaw
#include "math.h"

// Định nghĩa các trạng thái
typedef enum {
    STATE_RUN_STRAIGHT,
    STATE_TURN_90_1,
    STATE_MOVE_LANE,
    STATE_TURN_90_2
} RobotState_t;

// CÁC BIẾN NỘI BỘ (Chỉ dùng trong file này)
static RobotState_t current_state = STATE_RUN_STRAIGHT;
static int8_t turn_dir = -1;             // -1: Rẽ phải, 1: Rẽ trái
static f32 lane_distance = 0.30f;        // 30cm chuyển làn
static f32 start_lane_x = 0.0f;
static f32 start_lane_y = 0.0f;
static uint32_t state_start_time = 0;

void Robot_Task_Init(void) {
    Kinematics_reset();
    heading_target = yaw;
    current_state = STATE_RUN_STRAIGHT;
}

// HÀM HỖ TRỢ RẼ 90 ĐỘ (Nội bộ)
static void Robot_Turn_90(int8_t dir) {
    ph.x_t = pose.x;
    ph.y_t = pose.y;
    ph.theta_t = pose.theta + (dir * (PI / 2.0f));

    while(ph.theta_t > PI) ph.theta_t -= 2.0f * PI;
    while(ph.theta_t < -PI) ph.theta_t += 2.0f * PI;

    ph.state = PH_HOLD;
    g_sp_v = 0.0f;
}

// HÀM MÁY TRẠNG THÁI TỔNG (Gọi trong while(1))
void Robot_State_Machine(void) {
    f32 th_err, dx, dy, current_dist;

    switch(current_state) {

        case STATE_RUN_STRAIGHT:
            g_sp_v = 0.15f;

            if (Check_Front_Corridor()) {
                g_sp_v = 0.0f;
                Nav_Direction bypass_dir = Scan_and_Decide();

                if (bypass_dir == DIR_ABORT) {
                    Robot_Turn_90(turn_dir);
                    current_state = STATE_TURN_90_1;
                    state_start_time = HAL_GetTick();
                }
                else {
                    bool bypass_success = Execute_Obstacle_Bypass();
                    if (bypass_success) {
                        heading_target = yaw;
                    } else {
                        Robot_Turn_90(turn_dir);
                        current_state = STATE_TURN_90_1;
                        state_start_time = HAL_GetTick();
                    }
                }
            }
            break;

        case STATE_TURN_90_1:
            th_err = ph.theta_t - pose.theta;
            while(th_err > PI) th_err -= 2.0f * PI;
            while(th_err < -PI) th_err += 2.0f * PI;

            if (fabsf(th_err) < DEG2RAD(3.0f) || (HAL_GetTick() - state_start_time > 3000)) {
                PH_deactivate();
                heading_target = yaw;

                start_lane_x = pose.x;
                start_lane_y = pose.y;
                current_state = STATE_MOVE_LANE;
                state_start_time = HAL_GetTick();
            }
            break;

        case STATE_MOVE_LANE:
            g_sp_v = 0.20f;

            dx = pose.x - start_lane_x;
            dy = pose.y - start_lane_y;
            current_dist = sqrtf(dx * dx + dy * dy);

            if (Check_Front_Corridor() || current_dist >= lane_distance || (HAL_GetTick() - state_start_time > 4000)) {
                g_sp_v = 0.0f;
                Robot_Turn_90(turn_dir);
                current_state = STATE_TURN_90_2;
                state_start_time = HAL_GetTick();
            }
            break;

        case STATE_TURN_90_2:
            th_err = ph.theta_t - pose.theta;
            while(th_err > PI) th_err -= 2.0f * PI;
            while(th_err < -PI) th_err += 2.0f * PI;

            if (fabsf(th_err) < DEG2RAD(3.0f) || (HAL_GetTick() - state_start_time > 3000)) {
                PH_deactivate();
                heading_target = yaw;

                turn_dir = -turn_dir;
                current_state = STATE_RUN_STRAIGHT;
            }
            break;
    }
}

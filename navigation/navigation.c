/*
 * navigation.c
 */

#include "navigation.h"
#include "control.h" // Gọi biến pose, g_sp_v, ph...
#include "encoder.h" // Gọi biến ec_l, ec_r
#include "math.h"

// --- CÁC HẰNG SỐ TIMEOUT CHỐNG KẸT ---
#define TIMEOUT_MOVE_MS 4000  // Chờ tối đa 4 giây cho 1 pha rướn tịnh tiến
#define TIMEOUT_SCAN_MS 7000  // Chờ tối đa 7 giây cho pha men theo mép

uint16_t lidarDistance = 0;
uint16_t Lidar_Map[181] = {0};
static float nav_base_dist = 0.0f; // Mốc quãng đường nội bộ

// 1. GIAO TIẾP LIDAR CƠ BẢN
uint16_t Lidar_GetDist() {
    uint16_t current_distance = readRangeContinuousMillimeters(0);
    lidarDistance = current_distance;
    return current_distance;
}

// 2. GIAO TIẾP VỚI LỚP KINEMATICS & ĐỘNG CƠ
void Nav_Motor_Forward(void) {
    PH_deactivate();
    heading_target = yaw;
    g_sp_v = 0.15f;
}

void Nav_Motor_Stop(void) {
    g_sp_v = 0.0f;
    PH_activate(); // Neo cứng tọa độ
}

void Nav_MPU_Turn(int dir) {
    ph.x_t = pose.x;
    ph.y_t = pose.y;
    ph.theta_t = pose.theta + (dir * (PI / 2.0f));

    // Chuẩn hóa góc về [-PI, PI]
    while(ph.theta_t > PI) ph.theta_t -= 2.0f * PI;
    while(ph.theta_t < -PI) ph.theta_t += 2.0f * PI;

    ph.state = PH_HOLD;
    g_sp_v = 0.0f;

    // Đợi góc xoay hoàn tất (sai số < 3 độ)
    f32 th_err;
    uint32_t turn_start = HAL_GetTick();
    do {
        th_err = ph.theta_t - pose.theta;
        while(th_err >  PI) th_err -= 2.0f * PI;
        while(th_err < -PI) th_err += 2.0f * PI;
        HAL_Delay(10);
        if((HAL_GetTick() - turn_start) > 3000) break;
    } while (fabsf(th_err) > DEG2RAD(3.0f));

    PH_deactivate();
    heading_target = yaw;
}

void Nav_Encoder_Reset(void) {
    nav_base_dist = (ec_l.dist + ec_r.dist) / 2.0f;
}

float Nav_Encoder_Get_Dist(void) {
    float current_dist = (ec_l.dist + ec_r.dist) / 2.0f;
    return fabsf(current_dist - nav_base_dist);
}

// 3. QUÉT KHÔNG GIAN
bool Check_Front_Corridor(void) {
    for (int angle = 60; angle <= 120; angle += 10) {
        Servo_WriteAngle(angle);
        HAL_Delay(30);
        uint16_t dist = Lidar_GetDist();
        Lidar_Map[angle] = dist;
        if(dist > 0 && dist < SAFE_DISTANCE) {
            return true; // Phát hiện vật cản trong hành lang thẳng
        }
    }
    return false;
}

Nav_Direction Scan_and_Decide(void) {
    uint16_t left_clearance = 0, right_clearance = 0;
    int left_edge_angle = 0, right_edge_angle = 0;

    for(int i = 0; i <= 180; i++) Lidar_Map[i] = 0;

    for (int angle = 0; angle <= 180; angle += 10) {
        Servo_WriteAngle(angle);
        HAL_Delay(30);
        uint16_t dist = Lidar_GetDist();
        Lidar_Map[angle] = dist;

        if (angle >= 0 && angle <= 80) {
            if(dist > left_clearance) {
                right_clearance = dist;
                right_edge_angle = angle;
            }
        }
        else if(angle >= 100 && angle <= 180) {
            if(dist > left_clearance) {
                left_clearance = dist;
                left_edge_angle = angle;
            }
        }
    }

    Servo_WriteAngle(90);
    HAL_Delay(50);

    bool can_turn_left = ((left_clearance >= MIN_CLEARANCE) && (left_edge_angle >= 140));
    bool can_turn_right = ((right_clearance >= MIN_CLEARANCE) && (right_edge_angle <= 40));

    if(can_turn_left && can_turn_right) {
        return (left_clearance >= right_clearance) ? DIR_LEFT : DIR_RIGHT;
    }
    else if (can_turn_left) return DIR_LEFT;
    else if (can_turn_right) return DIR_RIGHT;

    return DIR_ABORT; // Không có lối lách
}

// 4. THỰC THI CHUYỂN LUỐNG & LÁCH VẬT CẢN (CÓ CHỐNG KẸT)
bool Execute_U_Turn(Nav_Direction turn_dir) {
    Nav_Motor_Stop();
    HAL_Delay(100);

    Nav_MPU_Turn(turn_dir); // Rẽ 90 độ

    Nav_Encoder_Reset();
    Nav_Motor_Forward();
    uint32_t start_time = HAL_GetTick();

    // Rướn 30cm để sang làn
    while (Nav_Encoder_Get_Dist() < 0.30f) {
        if (Check_Front_Corridor()) { Nav_Motor_Stop(); return false; }
        if ((HAL_GetTick() - start_time) > TIMEOUT_MOVE_MS) { Nav_Motor_Stop(); return false; }
        HAL_Delay(10);
    }
    Nav_Motor_Stop();

    Nav_MPU_Turn(turn_dir); // Rẽ 90 độ tiếp
    return true;
}

bool Execute_Obstacle_Bypass(void) {
    Nav_Motor_Stop();
    Servo_WriteAngle(90);
    HAL_Delay(100);

    uint16_t center_dist = Lidar_GetDist();
    Nav_Direction turn_dir = Scan_and_Decide();

    if (turn_dir == DIR_ABORT) return false;

    Nav_MPU_Turn(turn_dir);
    uint32_t start_time;

    if (center_dist > 0 && center_dist < SAFE_DISTANCE) {
        // --- BÁM SƯỜN THEO BIÊN DẠNG ĐỒ VẬT ---
        int side_angle = (turn_dir == DIR_LEFT) ? 0 : 180;
        float offset_dist = 0.0f;

        // [Pha 1: Dò ngang]
        Nav_Encoder_Reset();
        Nav_Motor_Forward();
        start_time = HAL_GetTick();

        bool edge_found = false;
        while (!edge_found) {
            if ((HAL_GetTick() - start_time) > TIMEOUT_SCAN_MS) { Nav_Motor_Stop(); Nav_MPU_Turn(-turn_dir); return false; }
            Servo_WriteAngle(side_angle);
            HAL_Delay(50);
            if (Lidar_GetDist() > (SAFE_DISTANCE + EDGE_JUMP_THRESHOLD)) edge_found = true;
            if (Check_Front_Corridor()) { Nav_Motor_Stop(); Nav_MPU_Turn(-turn_dir); return false; }
        }

        float target_dist = Nav_Encoder_Get_Dist() + CLEARANCE_RUN_M;
        start_time = HAL_GetTick();
        while (Nav_Encoder_Get_Dist() < target_dist) {
            if ((HAL_GetTick() - start_time) > TIMEOUT_MOVE_MS) break;
            if (Check_Front_Corridor()) { Nav_Motor_Stop(); Nav_MPU_Turn(-turn_dir); return false; }
            HAL_Delay(10);
        }
        Nav_Motor_Stop();
        offset_dist = Nav_Encoder_Get_Dist();

        // [Pha 2: Bám dọc]
        Nav_MPU_Turn(-turn_dir);
        Nav_Encoder_Reset();
        Nav_Motor_Forward();
        start_time = HAL_GetTick();

        edge_found = false;
        while (!edge_found) {
            if ((HAL_GetTick() - start_time) > TIMEOUT_SCAN_MS) { Nav_Motor_Stop(); return false; }
            Servo_WriteAngle(side_angle);
            HAL_Delay(50);
            if (Lidar_GetDist() > (SAFE_DISTANCE + EDGE_JUMP_THRESHOLD)) edge_found = true;
            if (Check_Front_Corridor()) { Nav_Motor_Stop(); return false; }
        }

        target_dist = Nav_Encoder_Get_Dist() + CLEARANCE_RUN_M;
        start_time = HAL_GetTick();
        while (Nav_Encoder_Get_Dist() < target_dist) {
            if ((HAL_GetTick() - start_time) > TIMEOUT_MOVE_MS) break;
            if (Check_Front_Corridor()) { Nav_Motor_Stop(); return false; }
            HAL_Delay(10);
        }
        Nav_Motor_Stop();

        // [Pha 3: Lùi về trục đường cũ]
        Nav_MPU_Turn(-turn_dir);
        Servo_WriteAngle(90);
        Nav_Encoder_Reset();
        Nav_Motor_Forward();
        start_time = HAL_GetTick();

        while (Nav_Encoder_Get_Dist() < offset_dist) {
            if ((HAL_GetTick() - start_time) > TIMEOUT_MOVE_MS) break;
            if (Check_Front_Corridor()) { Nav_Motor_Stop(); return false; }
            HAL_Delay(10);
        }
        Nav_Motor_Stop();

        Nav_MPU_Turn(turn_dir);
        return true;

    } else {
        // --- VƯỢT HỘP MÙ CỐ ĐỊNH ---
        Servo_WriteAngle(90);

        Nav_Encoder_Reset();
        Nav_Motor_Forward();
        start_time = HAL_GetTick();
        while (Nav_Encoder_Get_Dist() < 0.40f) {
            if ((HAL_GetTick() - start_time) > TIMEOUT_MOVE_MS) break;
            if (Check_Front_Corridor()) { Nav_Motor_Stop(); Nav_MPU_Turn(-turn_dir); return false; }
            HAL_Delay(10);
        }
        Nav_Motor_Stop();

        Nav_MPU_Turn(-turn_dir);
        Nav_Encoder_Reset();
        Nav_Motor_Forward();
        start_time = HAL_GetTick();
        while (Nav_Encoder_Get_Dist() < 0.50f) {
            if ((HAL_GetTick() - start_time) > TIMEOUT_MOVE_MS) break;
            if (Check_Front_Corridor()) { Nav_Motor_Stop(); return false; }
            HAL_Delay(10);
        }
        Nav_Motor_Stop();

        Nav_MPU_Turn(-turn_dir);
        Nav_Encoder_Reset();
        Nav_Motor_Forward();
        start_time = HAL_GetTick();
        while (Nav_Encoder_Get_Dist() < 0.40f) {
            if ((HAL_GetTick() - start_time) > TIMEOUT_MOVE_MS) break;
            if (Check_Front_Corridor()) { Nav_Motor_Stop(); return false; }
            HAL_Delay(10);
        }
        Nav_Motor_Stop();

        Nav_MPU_Turn(turn_dir);
        return true;
    }
}

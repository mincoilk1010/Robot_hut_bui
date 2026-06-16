/*
 * navigation.h
 * Thư viện xử lý Lidar, Servo và điều hướng tránh vật cản
 */

#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

// --- CÁC HẰNG SỐ ĐIỀU HƯỚNG ---
#define SAFE_DISTANCE          300   // Khoảng cách an toàn (mm) - Dưới mức này là vật cản
#define MIN_CLEARANCE          400   // Khoảng cách hở tối thiểu để lách (mm)
#define EDGE_JUMP_THRESHOLD    150   // Khoảng hở nhận diện mép vật cản (mm)
#define CLEARANCE_RUN_M        0.28f // Khoảng cách rướn qua mép vật cản (28cm)

// --- KIỂU DỮ LIỆU ---
typedef enum {
    DIR_LEFT = 1,
    DIR_RIGHT = -1,
    DIR_ABORT = 0
} Nav_Direction;

// --- BIẾN TOÀN CỤC ---
extern uint16_t lidarDistance;
extern uint16_t Lidar_Map[181];

// --- KHAI BÁO HÀM ---
// 1. Hàm đọc Lidar
uint16_t Lidar_GetDist(void);

// 2. Giao tiếp cơ sở hạ tầng (Gọi xuống Control/Encoder)
void Nav_Motor_Forward(void);
void Nav_Motor_Stop(void);
void Nav_MPU_Turn(int dir);
void Nav_Encoder_Reset(void);
float Nav_Encoder_Get_Dist(void);

// 3. Hàm phân tích không gian
bool Check_Front_Corridor(void);
Nav_Direction Scan_and_Decide(void);

// 4. Hàm thực thi cấp cao
bool Execute_U_Turn(Nav_Direction turn_dir);
bool Execute_Obstacle_Bypass(void);

// Lưu ý: Đảm bảo bạn đã khai báo hàm Servo_WriteAngle(angle) và readRangeContinuousMillimeters() ở thư viện của Lidar/Servo
extern void Servo_WriteAngle(int angle);
extern uint16_t readRangeContinuousMillimeters(int mode);

#endif /* NAVIGATION_H_ */

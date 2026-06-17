#ifndef __NAVIGATION_H
#define __NAVIGATION_H

#include "types.h"	
#include <stdint.h>
#include <stdbool.h>
#include "VL53L0X.h" // Thêm thư viện cảm biến Lidar VL53L0X để lấy giá trị khoảng cách
#include "Servo.h" // Thêm thư viện điều khiển servo nếu cần thiết cho việc quay lái

#define SAFE_DISTANCE 200        // Khoảng cách an toàn để di chuyển thẳng (20cm)
#define EDGE_JUMP_THRESHOLD 150  // Khoảng cách bước nhảy để phát hiện cạnh (15cm)
#define MIN_CLEARANCE 300        // Khe ho phai sau > 30cm moi lot xe
#define ROBOT_WIDTH_M 0.27f //Coi robot la hinh vuong co canh 27cm de tinh toan khoang cach toi mep trai va phai
#define CLEARANCE_RUN_M 0.28f //Quãng đường chạy rướn thoát thân 27cm đuôi xe + 8cm bảo hiểm (Do cảm biến ở mũi)


// Giữ nguyên Enum chuẩn của bạn để quản lý Máy trạng thái (State Machine)
typedef enum {
    ROBOT_FORWARD,          
    ROBOT_BACKWARD_30CM,    
    ROBOT_TURN_RIGHT_90,    
    ROBOT_SIDE_FORWARD_30,  
    ROBOT_TURN_LEFT_90,     
} RobotState_t;

typedef enum {
    DIR_LEFT = -90,
    DIR_RIGHT = 90,
    DIR_ABORT = 0
} Nav_Direction;

// Cấu hình tham số điều khiển xoay bằng MPU6050 (dùng g_sp_w, không dùng motor_run)
#define TURN_W_SPEED      1.8f    // rad/s — toc do goc khi xoay binh thuong
#define TURN_W_SLOW       0.6f    // rad/s — toc do goc khi gan dich (giam overshoot)
#define TURN_SLOW_DEG     20.0f   // do — nguong bat dau giam toc
#define TURN_DONE_DEG     1.5f    // do — sai so coi la xong
#define TURN_SETTLE_TICKS 8u      // so lan poll x 5ms = 40ms on dinh moi thoat

#define NAV_FWD_SPEED     0.15f   // m/s — toc do tien thang

extern uint16_t lidarDistance; // Biến toàn cục lưu giá trị khoảng cách đo được từ Lidar
extern uint16_t Lidar_Map[181]; // Bản đồ khoảng cách từ 0 đến 180 độ, mỗi phần tử lưu khoảng cách tại góc tương ứng


// Khai báo các hàm di chuyển cơ bản (Primitive API)
//Ham tra ve gia tri distance cua VL53L0X
uint16_t Lidar_GetDist(void);

//Phan di chuyen
void Nav_Motor_Forward(void); //Chay tien
void Nav_Motor_Stop(void);  //Phanh
void Nav_MPU_Turn(int angle); //Quay xe
void Nav_Encoder_Reset(void); //Reset encoder ve 0
float Nav_Encoder_Get_Dist(void); //Lay gia tri encoder da di duoc tinh tu luc reset

//Kich ban test navigation
void test_robot_navigation(void);

//Phan ra quyet dinh
bool Check_Front_Corridor(void);
Nav_Direction Scan_and_Decide(void);
bool Execute_Obstacle_Bypass(void);

#endif /* __NAVIGATION_H */
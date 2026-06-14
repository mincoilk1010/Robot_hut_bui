
#ifndef INC_NAVIGATION_H_
#define INC_NAVIGATION_H_

#include <stdint.h>
#include <stdbool.h>
#include "VL53L0X.h" // Thêm thư viện cảm biến Lidar VL53L0X để lấy giá trị khoảng cách
#include "Servo.h" // Thêm thư viện điều khiển servo nếu cần thiết cho việc quay lái


//Cac hang so hinh hoc va nguong thuat toan de quyet dinh huong di chuyen
#define SAFE_DISTANCE 200        // Khoảng cách an toàn để di chuyển thẳng (20cm)
#define EDGE_JUMP_THRESHOLD 150  // Khoảng cách bước nhảy để phát hiện cạnh (15cm)
#define MIN_CLEARANCE 300        // Khe ho phai sau > 30cm moi lot xe
#define ROBOT_WIDTH_M 0.27f //Coi robot la hinh vuong co canh 27cm de tinh toan khoang cach toi mep trai va phai
#define CLEARANCE_RUN_M 0.28f //Quãng đường chạy rướn thoát thân 27cm đuôi xe + 8cm bảo hiểm (Do cảm biến ở mũi)

extern uint16_t lidarDistance; // Biến toàn cục lưu giá trị khoảng cách đo được từ Lidar
extern uint16_t Lidar_Map[181]; // Bản đồ khoảng cách từ 0 đến 180 độ, mỗi phần tử lưu khoảng cách tại góc tương ứng

// Khai báo các hướng quyết định
typedef enum {
    DIR_LEFT = -90,
    DIR_RIGHT = 90,
    DIR_ABORT = 0
} Nav_Direction;

//Ham tra ve gia tri distance cua VL53L0X
uint16_t Lidar_GetDist(void);

//Phan di chuyen
void Nav_Motor_Forward(void); //Chay tien
void Nav_Motor_Stop(void);  //Phanh
void Nav_MPU_Turn(int angle); //Quay xe
void Nav_Encoder_Reset(void); //Reset encoder ve 0
float Nav_Encoder_Get_Dist(void); //Lay gia tri encoder da di duoc tinh tu luc reset

//Phan ra quyet dinh
bool Check_Front_Corridor(void);
Nav_Direction Scan_and_Decide(void);
bool Execute_Obstacle_Bypass(void);

#endif /* INC_NAVIGATION_H_ */
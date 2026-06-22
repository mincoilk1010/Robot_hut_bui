#include "navigation.h"
#include "control.h"     /* g_sp_v, g_sp_w, turn_start/task/done, heading_target */
#include "encoder.h"     /* ec_l, ec_r */
#include "mpu6050.h"     /* yaw */
#include "nav.h"         /* NAV_SPEED */

uint16_t lidarDistance = 0; // Mang luu gia tri VL53L0X o cac goc quay
uint16_t Lidar_Map[181] = {0}; // Ban do luu gia tri VL53L0X o cac goc quay tu 0 den 180 do

/* Snapshot encoder tại lúc gọi Nav_Encoder_Reset() */
static float _enc_snap_l = 0.0f;
static float _enc_snap_r = 0.0f;


//Ham lay gia tri khoang cach hien tai va luu vao bien toan cuc (de sau nay hien thi ra OLED)
uint16_t Lidar_GetDist() {
    uint16_t current_distance = readRangeContinuousMillimeters(0); // Ham doc gia tri tu VL53L0X
    lidarDistance = current_distance; // Cap nhat gia tri vao bien toan cuc
    return current_distance;
}

//=====================================================DI CHUYEN===============================================================

//=============================================================================================================================
void Nav_Motor_Forward(void) {
    // Để trống hoặc viết code điều khiển motor tiến ở đây sau
		heading_target = yaw;
    g_sp_v = NAV_SPEED;
    g_sp_w = 0.0f;

}

void Nav_Motor_Reverse(void) {
    // Để trống hoặc viết code điều khiển motor lùi ở đây sau
		heading_target = yaw;
    g_sp_v = -NAV_SPEED;
    g_sp_w = 0.0f;
}

void Nav_Motor_Stop(void) {
    g_sp_v = 0.0f;
    g_sp_w = 0.0f;
    HAL_Delay(50);
}

void Nav_MPU_Turn(int angle)
{
    if (angle == 0) return;
 
    /* Dừng hoàn toàn trước khi quay để tránh trượt bánh */
    g_sp_v = 0.0f;
    g_sp_w = 0.0f;
    HAL_Delay(50);
 
    turn_start((float)angle);
 
    /* Polling — ISR lo turn_task() + motorcontrol_pid() mỗi 20ms */
    uint32_t t0 = HAL_GetTick();
    while (!turn_done())
    {
        if ((HAL_GetTick() - t0) > (TURN_TIMEOUT_MS + 500u))
        {
            g_sp_v = 0.0f;
            g_sp_w = 0.0f;
            break;  /* timeout an toàn */
        }
        /* Main loop rảnh → có thể đọc sensor, log UART, v.v. */
    }
 
    /* Dừng sạch sau khi quay xong */
    g_sp_v = 0.0f;
    g_sp_w = 0.0f;
    HAL_Delay(50);
 
    /* Chốt heading mới cho lần Forward tiếp theo */
    heading_target = yaw;
}

void Nav_Encoder_Reset(void)
{
    _enc_snap_l = ec_l.dist;
    _enc_snap_r = ec_r.dist;
}

float Nav_Encoder_Get_Dist(void)
{
    float dl = ec_l.dist - _enc_snap_l;
    float dr = ec_r.dist - _enc_snap_r;
    return (dl + dr) * 0.5f;
}



//Quet hanh lang phia truoc (60-120 do)
bool Check_Front_Corridor(void) {
    // Quét từ 60 đến 120 độ, kiểm tra nếu tất cả đều > SAFE_DISTANCE thì trả về true
    for (int angle = 60; angle <= 120; angle += 10) { 
        Servo_WriteAngle(angle); // Quay servo đến góc cần quét
        HAL_Delay(30); // Chờ servo ổn định

        uint16_t dist = Lidar_GetDist(); // Đọc khoảng cách tại góc này
        Lidar_Map[angle] = dist; // Cap nhat vao mang 

        if(dist > 0 && dist < SAFE_DISTANCE) {
            return true; // Phát hiện vật cản trong hành lang
        }
    }
    return false; // Hành lang phía trước an toàn
}

//=====================================================BO LOC KHONG GIAN==============================================================

//=============================================================================================================================
Nav_Direction Scan_and_Decide(void) {
    uint16_t left_clearance = 0, right_clearance = 0;
    int left_edge_angle = 0, right_edge_angle = 0;

    //RESET Lidar_Map truoc khi quet lai
    for(int i=0; i<=180; i++) {
        Lidar_Map[i] = 0;
    }

    for (int angle = 0; angle <= 180; angle += 10) {
        Servo_WriteAngle(angle);
        HAL_Delay(30); // Chờ servo ổn định
        uint16_t dist = Lidar_GetDist();
        Lidar_Map[angle] = dist; // Cap nhat vao mang

        //Phan tich diem sau nhat
        //check ben phai
        if (angle >=0 && angle <= 80) {
            if(dist > right_clearance) {
                right_clearance = dist;
                right_edge_angle = angle; 
            }
        }
        //check ben trai
        else if(angle >=100 && angle <= 180) {
            if(dist > left_clearance) {
                left_clearance = dist;
                left_edge_angle = angle;
            }
        }
    }
    Servo_WriteAngle(90);
    HAL_Delay(50);

    /*Ap dung mang loc an toan: ket hop giua khoang cach va goc*/
    bool can_turn_left = ((left_clearance >= MIN_CLEARANCE) && (left_edge_angle >= 140));
    bool can_turn_right = ((right_clearance >= MIN_CLEARANCE) && (right_edge_angle <= 40));

    if(can_turn_left && can_turn_right) {
        return (left_clearance >= right_clearance) ? DIR_LEFT : DIR_RIGHT;
    } 
    else if (can_turn_left) {
        return DIR_LEFT;
    }
    else if (can_turn_right) {
        return DIR_RIGHT;
    }
    return DIR_ABORT; //Khong the vuot qua vat can, quay dau
}



//THUC THI DIEU HUONG NE VAT CAN
bool Execute_Obstacle_Bypass(void) {
    Nav_Motor_Stop();
    
    Servo_WriteAngle(90);
    HAL_Delay(100);
    uint16_t center_dist = Lidar_GetDist();

    Nav_Direction turn_dir = Scan_and_Decide();
    
    if (turn_dir == DIR_ABORT) {
        return false; // Ép gọi U-Turn trên hàm main
    }

    Nav_MPU_Turn(turn_dir); // Bẻ vô lăng toàn thân xe

    // PHÂN LOẠI VẬT CẢN (Tâm vs Lệch biên)
    if (center_dist > 0 && center_dist < SAFE_DISTANCE) {
        
        // --- KỊCH BẢN 1: BÁM SƯỜN ---
        // Nếu rẽ Trái -> Vật cản nằm bên Phải -> Servo quay góc 0 để nhìn Phải
        // Nếu rẽ Phải -> Vật cản nằm bên Trái -> Servo quay góc 180 để nhìn Trái
        int side_angle = (turn_dir == DIR_LEFT) ? 0 : 180;
        float offset_dist = 0.0f;
        
        // [Pha 1: Dò ngang]
        Nav_Encoder_Reset();
        Nav_Motor_Forward();
        
        bool edge_found = false;
        while (!edge_found) {
            Servo_WriteAngle(side_angle);
            HAL_Delay(50);
            if (Lidar_GetDist() > (SAFE_DISTANCE + EDGE_JUMP_THRESHOLD)) {
                edge_found = true;
            }
            if (Check_Front_Corridor()) {
                Nav_Motor_Stop();
                Nav_MPU_Turn(-turn_dir); 
                return false; 
            }
        }
        
        // Rướn 28cm
        float target_dist = Nav_Encoder_Get_Dist() + CLEARANCE_RUN_M;
        while (Nav_Encoder_Get_Dist() < target_dist) {
            if (Check_Front_Corridor()) {
                Nav_Motor_Stop();
                Nav_MPU_Turn(-turn_dir); 
                return false; 
            }
            HAL_Delay(10);
        }
        Nav_Motor_Stop();
        offset_dist = Nav_Encoder_Get_Dist(); 
        
        // [Pha 2: Bám dọc]
        Nav_MPU_Turn(-turn_dir); 
        Nav_Encoder_Reset();
        Nav_Motor_Forward();
        
        edge_found = false;
        while (!edge_found) {
            Servo_WriteAngle(side_angle);
            HAL_Delay(50);
            if (Lidar_GetDist() > (SAFE_DISTANCE + EDGE_JUMP_THRESHOLD)) {
                edge_found = true;
            }
            if (Check_Front_Corridor()) {
                Nav_Motor_Stop();
                return false; 
            }
        }
        
        // Rướn 28cm lần hai
        target_dist = Nav_Encoder_Get_Dist() + CLEARANCE_RUN_M;
        while (Nav_Encoder_Get_Dist() < target_dist) {
            if (Check_Front_Corridor()) {
                Nav_Motor_Stop();
                return false; 
            }
            HAL_Delay(10);
        }
        Nav_Motor_Stop();
        
        // [Pha 3: Lùi về trục đường cũ]
        Nav_MPU_Turn(-turn_dir);
        Servo_WriteAngle(90); 
        Nav_Encoder_Reset();
        Nav_Motor_Forward();
        while (Nav_Encoder_Get_Dist() < offset_dist) {
            if (Check_Front_Corridor()) {
                Nav_Motor_Stop();
                return false; 
            }
            HAL_Delay(10);
        }
        Nav_Motor_Stop();
        
        Nav_MPU_Turn(turn_dir);
        return true; 
        
    } else {
        
        // --- KỊCH BẢN 2: LƯỚI CỐ ĐỊNH (Hộp mù) ---
        Servo_WriteAngle(90); 
        
        // [Dạt 40cm]
        Nav_Encoder_Reset();
        Nav_Motor_Forward();
        while (Nav_Encoder_Get_Dist() < 0.40f) {
            if (Check_Front_Corridor()) {
                Nav_Motor_Stop();
                Nav_MPU_Turn(-turn_dir); 
                return false; 
            }
            HAL_Delay(10);
        }
        Nav_Motor_Stop();
        
        // [Tiến 50cm]
        Nav_MPU_Turn(-turn_dir);
        Nav_Encoder_Reset();
        Nav_Motor_Forward();
        while (Nav_Encoder_Get_Dist() < 0.50f) {
            if (Check_Front_Corridor()) {
                Nav_Motor_Stop();
                return false; 
            }
            HAL_Delay(10);
        }
        Nav_Motor_Stop();
        
        // [Lùi 40cm về luống]
        Nav_MPU_Turn(-turn_dir);
        Nav_Encoder_Reset();
        Nav_Motor_Forward();
        while (Nav_Encoder_Get_Dist() < 0.40f) {
            if (Check_Front_Corridor()) {
                Nav_Motor_Stop();
                return false; 
            }
            HAL_Delay(10);
        }
        Nav_Motor_Stop();
        
        Nav_MPU_Turn(turn_dir);
        return true;
    }
}
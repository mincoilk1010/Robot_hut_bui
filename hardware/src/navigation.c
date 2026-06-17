#include "navigation.h"
#include "motor.h"
#include "mpu6050.h"
#include "encoder.h"
#include "control.h"
#include "delay.h"
#include "math.h"
#include "Servo.h"
#include "VL53L0X.h"
// ===================================================CONFIG CAC GIA TRI ======================================================================


uint16_t lidarDistance = 0; // Mang luu gia tri VL53L0X o cac goc quay
uint16_t Lidar_Map[181] = {0}; // Ban do luu gia tri VL53L0X o cac goc quay tu 0 den 180 do

/* Cờ "lái thủ công": khi =1, motorcontrol_pid() ở control.c sẽ KHÔNG
 * ghi vào TIM4 nữa, để Nav_MPU_Turn() được toàn quyền điều khiển động cơ
 * bằng motor_run() trong lúc xoay. */
volatile u8 nav_manual = 0;


//Ham lay gia tri khoang cach hien tai va luu vao bien toan cuc (de sau nay hien thi ra OLED)
uint16_t Lidar_GetDist() {
    uint16_t current_distance = readRangeContinuousMillimeters(0); // Ham doc gia tri tu VL53L0X
    lidarDistance = current_distance; // Cap nhat gia tri vao bien toan cuc
    return current_distance;
}

// ===================================================NHUNG HAM DI CHUYEN CO BAN=====================================================================

/* Chạy thẳng: chốt hướng hiện tại làm hướng chuẩn, đặt vận tốc mục tiêu.
 * Vòng lặp nền (HeadingHold_Task + motorcontrol_pid, chạy mỗi 20ms trong
 * ngắt MPU6050) sẽ tự bơm PWM ra TIM4 và giữ xe đi thẳng theo yaw. */

void Nav_Motor_Forward(void) {  /* Bật cầu H chạy thẳng */ 

    nav_manual = 0;

    PID_Reset(&pid_l);
    PID_Reset(&pid_r);

    heading_target = yaw;   // "khóa" hướng đi hiện tại
    g_sp_w = 0.0f;
    g_sp_v = NAV_FWD_SPEED;


}


void Nav_Motor_Stop(void)    { /* Phanh động cơ */ 
	
}

/*
 * Xoay xe tai cho mot goc `angle` (do) so voi huong hien tai.
 * Su dung g_sp_w de dat toc do goc, motorcontrol_pid() + PID van toc
 * (chay trong ngat MPU 10ms) tu dieu khien PWM — KHONG dung motor_run().
 *
 * angle > 0 (DIR_RIGHT = +90): xoay phai
 * angle < 0 (DIR_LEFT  = -90): xoay trai
 */
void Nav_MPU_Turn(int angle)
{
	turn_start(angle);
	turn_task();
}


void Nav_Encoder_Reset(void) { /* Xóa biến đếm số vòng bánh xe */ 
   
}


float Nav_Encoder_Get_Dist(void) {  
    return (ec_l.dist + ec_r.dist) * 0.5f;
}

// ===================================================TEST NAVIGATION=====================================================================


void test_robot_navigation(void)
{
    

}

//==================================================== Bo loc khong gian==============================================================================


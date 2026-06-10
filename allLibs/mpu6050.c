/*
 * mpu6050.c
 *
 *  Created on: Jun 7, 2026
 *      Author: GB Center
 */


#include "mpu6050.h"

//Goi cau hinh i2c tu main.c
extern I2C_HandleTypeDef hi2c1;

int16_t gz = 0;
float GZ = 0.0f;
float GZ_calib = 0.0f;
volatile float yaw = 0.0f;

void mpu6050_Init(void)
{
    uint8_t check, mData;
 
    HAL_Delay(100);  // Ch? MPU ?n d?nh
 
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 10);
    if (check != 0x68) return; 
 
    // Wake up, dùng PLL t? Gyro-X d? có clock ?n d?nh
    mData = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &mData, 1, 10);
    HAL_Delay(10);
 
    // SMPLRT_DIV = 9 -> Sample Rate = 1kHz / (1 + 9) = 100Hz (Chu k? ng?t 10ms)
    mData = 0x09;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x19, 1, &mData, 1, 10);
 
    // DLPF bandwidth ~44Hz - L?c nhi?u rung d?ng co bám sàn
    mData = 0x03;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1A, 1, &mData, 1, 10);
 
    // Gyro FS = ±250°/s -> Sensitivity 131 LSB/(°/s) - Ð? phân gi?i cao nh?t cho robot xoay
    mData = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, 1, &mData, 1, 10);
        
    // B?t ng?t Data Ready (Thanh ghi 0x38), s?a timeout thành 10 nhu dã fix
    mData = 0x01; 
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x38, 1, &mData, 1, 10);
}

// Ham hieu chuan: goi ham nay khi vua bat nguon
void mpu6050_Calibrate(void)
{
    long sumGZ = 0;
    // Ð?c 2000 l?n l?y trung bình nhi?u tinh tr?c Z
    for (int i = 0; i < 2000; i++) {
        uint8_t gy_data[2];
        // Ch? d?c riêng 2 thanh ghi c?a tr?c Z (0x47 và 0x48) thay vì d?c c? 6 thanh ghi
        HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x47, 1, gy_data, 2, 10);
        int16_t gz_raw = (int16_t)(gy_data[0] << 8 | gy_data[1]);
        sumGZ += gz_raw;
        HAL_Delay(1);
    }
    GZ_calib = (float)(sumGZ / 2000.0f) / 131.0f;
    yaw = 0.0f; // Reset góc robot v? v? trí 0 ban d?u
}


// T?I UU: Ch? d?c dúng 2 byte d? li?u v?n t?c góc c?a tr?c Z (GYRO_ZOUT)
void mpu6050_readGyroZ(void)
{
    uint8_t gy_data[2];
    // Ð?a ch? thanh ghi d? li?u Tr?c Z c?a Gyro là 0x47
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x47, 1, gy_data, 2, 10);
 
    gz = (int16_t)(gy_data[0] << 8 | gy_data[1]);
    GZ = (float)gz / 131.0f;
}



// T?I UU: Lo?i b? hoàn toàn b? l?c bù ph?c t?p, ch? tính tích phân Yaw t?c d? cao
void mpu6050_processYaw(float dt)
{
    // Tr? sai s? tinh ch?ng trôi góc
    float gz_corrected = GZ - GZ_calib;
 
    // B? l?c vùng ch?t (Dead-zone) tránh trôi góc khi robot d?ng yên
    if (gz_corrected > -0.05f && gz_corrected < 0.05f) {
        gz_corrected = 0.0f;
    }
 
    // Tích phân Gyro Z theo th?i gian th?c d? ra góc Yaw
    yaw += gz_corrected * dt;
 
    // Chu?n hóa góc v? kho?ng [-180, 180] d? ph?c v? thu?t toán di chuy?n
    if (yaw >  180.0f) yaw -= 360.0f;
    if (yaw < -180.0f) yaw += 360.0f;
}

/* ---------------------------------------------------------------
 * Tính góc lech ngan nhat gia 2 góc (xu lý wrap-around ±180°)
 * Ví du: angleDiff(170°, -170°) = -20°
 * --------------------------------------------------------------- */
/* Tính góc l?ch ng?n nh?t ph?c v? thu?t toán PID di?u hu?ng hu?ng di c?a robot */
float mpu6050_angleDiff(float target, float current)
{
    float diff = target - current;
    if (diff >  180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    return diff;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == MPU6050_PIN_INT) 
    {
        float dt = 0.01f; // Chu k? 10ms chu?n xác t? ph?n c?ng MPU

        mpu6050_readGyroZ();   // Ch? d?c 2 byte tr?c Z qua I2C (C?c nhanh)
        mpu6050_processYaw(dt); // C?ng d?n góc xoay
    }
}

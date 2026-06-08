/*
 * mpu6050.c
 *
 *  Created on: Jun 7, 2026
 *      Author: GB Center
 */


#include "mpu6050.h"

//Goi cau hinh i2c tu main.c
I2C_HandleTypeDef hi2c1;

// Dinh nghia cac bien toan cuc
int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
float AX = 0.0f, AY = 0.0f, AZ = 0.0f, GX = 0.0f, GY = 0.0f, GZ = 0.0f;

volatile float pitch = 0.0f;
volatile float roll = 0.0f;
volatile float yaw = 0.0f;				//luu gia tri goc quay hien tai cua robot
float GZ_calib = 0.0f;		//luu gia tri nhieu truc Z khi dung yen

void mpu6050_Init(void)
{
    uint8_t check, mData;

    HAL_Delay(100);  // Cho mpu on dinh

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 10);
    if (check != 0x68) return;   // WHO_AM_I sai ? return

    // Wake up, dùng PLL tu Gyro-X on dinh hon internal oscillator)
    mData = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &mData, 1, 10);
    HAL_Delay(10);

    // SMPLRT_DIV = 9 -> Sample Rate = 1kHz / (1 + 9) (vòng lap chay ~100Hz chu ky 10ms
    mData = 0x09;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x19, 1, &mData, 1, 10);

    // DLPF bandwidth ~44Hz - loc rung dong motor/sàn nhà
    mData = 0x03;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1A, 1, &mData, 1, 10);

    // Gyro FS = ±250°/s -> sensitivity 131 LSB/(°/s) - chính xác nhat
    mData = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, 1, &mData, 1, 10);

    // Accel FS = ±2g -> sensitivity 16384 LSB/g
    mData = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, 1, &mData, 1, 10);

		// Interrupt
    mData = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x38, 1, &mData, 1, 10);
}

// Ham hieu chuan: goi ham nay khi vua bat nguon
void mpu6050_Calibrate(void){
    long sumGZ = 0;
    // doc 2000 lan de lay gia tri trung binh sai so tinh
    for (int i = 0; i < 2000; i++) {
        uint8_t gy_data[6];
        HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, 1, gy_data, 6, 10);
        int16_t gz_raw = (int16_t)(gy_data[4] << 8 | gy_data[5]);
        sumGZ += gz_raw;
        HAL_Delay(1);
    }
    // Tinh toan gi tri offset sang float
    GZ_calib = (float)(sumGZ / 2000.0f) / 131.0f;
		// Reset yaw ve 0 sau khi calibrate
    yaw = 0.0f;
}


//Doc data van toc goc GYROSCOPE
void mpu6050_readGyro(void)
{
    uint8_t gy_data[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, 1, gy_data, 6, 10);

    gx = (int16_t)(gy_data[0] << 8 | gy_data[1]);
    gy = (int16_t)(gy_data[2] << 8 | gy_data[3]);
    gz = (int16_t)(gy_data[4] << 8 | gy_data[5]);

    GX = (float)gx / 131.0f;
    GY = (float)gy / 131.0f;
    GZ = (float)gz / 131.0f;
}


/**
 Doc data gia toc goc (Accelerometer)
  */
void mpu6050_readAccl(void)
{
    uint8_t accl_data[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, accl_data, 6, 10);

    ax = (int16_t)(accl_data[0] << 8 | accl_data[1]);
    ay = (int16_t)(accl_data[2] << 8 | accl_data[3]);
    az = (int16_t)(accl_data[4] << 8 | accl_data[5]);

    AX = (float)ax / 16384.0f;
    AY = (float)ay / 16384.0f;
    AZ = (float)az / 16384.0f;
}


//Bo loc bu, tinh toan goc pitch va roll
void mpu6050_filter(float dt)
{
    // --- Pitch & Roll: Complementary Filter (Gyro + Accel bù) ---
    float pitch_Gy = pitch + GX * dt;
    float roll_Gy  = roll  + GY * dt;

    float pitch_Accl = atan2f(AY, sqrtf(AX*AX + AZ*AZ)) * RTD;
    float roll_Accl  = atan2f(AX, sqrtf(AY*AY + AZ*AZ)) * RTD;

    pitch = 0.98f * pitch_Gy + 0.02f * pitch_Accl;
    roll  = 0.98f * roll_Gy  + 0.02f * roll_Accl;


    // FIX: tru GZ_calib de loai offset nhieu tinh - dây là loi chính gây drift nhanh
    float gz_corrected = GZ - GZ_calib;

    // Dead-zone: neu cam bien dung ten ma Gyro van ra gia tri nho -> coi = 0
    // Nguong 0.05°/s tuong duong ~0.05°/s * 3600s = 180° drift sau 1 tieng dung yên
    if (gz_corrected > -0.05f && gz_corrected < 0.05f) {
        gz_corrected = 0.0f;
    }

    yaw += gz_corrected * dt;

    // Chuan hóa ve [-180, 180]
    if (yaw >  180.0f) yaw -= 360.0f;
    if (yaw < -180.0f) yaw += 360.0f;
}

/* ---------------------------------------------------------------
 * Tính góc lech ngan nhat gia 2 góc (xu lý wrap-around ±180°)
 * Ví du: angleDiff(170°, -170°) = -20°
 * --------------------------------------------------------------- */
float mpu6050_angleDiff(float target, float current)
{
    float diff = target - current;
    if (diff >  180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    return diff;
}



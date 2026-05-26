#include "mpu6050.h"

//Goi cau hinh i2c tu main.c
extern I2C_HandleTypeDef hi2c1;

// Dinh nghia cac bien toan cuc
int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
float AX = 0.0f, AY = 0.0f, AZ = 0.0f, GX = 0.0f, GY = 0.0f, GZ = 0.0f;

float pitch = 0.0f;
float roll = 0.0f;

void mpu6050_Init(void){
    uint8_t check;
    uint8_t mData;
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 1);
    if(check == 0x68){
        mData = 0x00; // Wake up MPU6050
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &mData, 1, 1);
        mData = 0x07; // Sample Rate Divider
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x19, 1, &mData, 1, 1);
        mData = 0x00; // Gyro Config
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, 1, &mData, 1, 1);
        mData = 0x00; // Accel Config
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, 1, &mData, 1, 1);
    }
}

//Doc data van toc goc GYROSCOPE
void mpu6050_readGyro(void){
    uint8_t gy_data[6];
    
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, 1, gy_data, 6, 1000);
    
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
void mpu6050_readAccl(void){
    uint8_t Accl_data[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, Accl_data, 6, 1000);
    
    ax = (int16_t)(Accl_data[0] << 8 | Accl_data[1]);
    ay = (int16_t)(Accl_data[2] << 8 | Accl_data[3]);
    az = (int16_t)(Accl_data[4] << 8 | Accl_data[5]);
    
    AX = (float)ax / 16384.0f;
    AY = (float)ay / 16384.0f;
    AZ = (float)az / 16384.0f;
}

//Bo loc bu, tinh toan goc pitch va roll
void mpu6050_filter(void){
    float dt = 0.003f; // Uoc luong chu ki vong lap thuc te ~3ms
    
    float pitch_Gy = pitch + GX * dt;
    float roll_Gy = roll + GY * dt;
    
    float pitch_Accl = atan2(AY, sqrt(AX*AX + AZ*AZ)) * RTD;
    float roll_Accl = atan2(AX, sqrt(AY*AY + AZ*AZ)) * RTD;
    
    // Dong bo lai he so loc bu chuan xac cho ca 2 truc
    pitch = 0.98f * pitch_Gy + 0.02f * pitch_Accl;
    roll  = 0.98f * roll_Gy  + 0.02f * roll_Accl;
}
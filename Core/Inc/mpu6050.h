#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f103xb.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal.h"

/* 1. ĐỊNH NGHĨA ĐỊA CHỈ I2C VÀ THANH GHI MPU6050 */
#define MPU6050_I2C_ADDR         (0x68 << 1) // Địa chỉ I2C mặc định (dịch trái 1 bit cho HAL)

#define MPU6050_REG_WHO_AM_I     0x75
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H  0x43

/* 2. CẤU TRÚC LƯU TRỮ DỮ LIỆU CẢM BIẾN */
typedef struct {
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gyro_Y;
    double Gz;
    
    float Temperature;
} MPU6050_t;

/* 3. NGUYÊN MẪU CÁC HÀM (API) */
// Hàm khởi tạo cảm biến (Cấu hình nguồn, dải đo...)
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);

// Hàm đọc toàn bộ dữ liệu thô và chuyển đổi sang đơn vị vật lý (g, deg/s)
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct);

#endif /* __MPU6050_H */
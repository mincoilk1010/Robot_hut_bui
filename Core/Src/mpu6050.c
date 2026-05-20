#include "mpu6050.h"

uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t check;
    uint8_t data;

    // Kiểm tra xem MPU6050 có phản hồi đúng mã định danh không
    HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDR, MPU6050_REG_WHO_AM_I, 1, &check, 1, 100);

    if (check == 0x68) { // 0x68 là giá trị mặc định của thanh ghi WHO_AM_I
        // Đánh thức cảm biến (Ghi 0 vào thanh ghi PWR_MGMT_1)
        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR, MPU6050_REG_PWR_MGMT_1, 1, &data, 1, 100);
        
        // Bạn có thể cấu hình thêm dải đo gia tốc (tại đây)
        return 0; // Khởi tạo thành công
    }
    return 1; // Khởi tạo thất bại
}

void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[14];

    // Đọc liên tục 14 byte dữ liệu từ thanh ghi ACCEL_XOUT_H (gồm: 6 byte Accel, 2 byte Temp, 6 byte Gyro)
    HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_XOUT_H, 1, Rec_Data, 14, 100);

    // Ghép 2 byte (High và Low) thành số nguyên 16-bit có dấu
    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    
    // Giả sử dải đo mặc định là +/- 2g (hệ số chia là 16384.0)
    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / 16384.0;

    // Tương tự giải mã cho Gyro và Temperature...
    // DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
}
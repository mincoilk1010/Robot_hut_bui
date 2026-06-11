/*
 * mpu6050.c
 *
 *  Created on: Jun 7, 2026
 *      Author: GB Center
 */


#include "mpu6050.h"
#include "control.h"
#include "types.h"

//Goi cau hinh i2c tu main.c


int16_t gz = 0;
float GZ = 0.0f;
float GZ_calib = 0.0f;
volatile float yaw = 0.0f;
_vo u8 Flag = 0;
void mpu6050_Init(void)
{
    uint8_t check, mData;

    HAL_Delay(100);  // Chờ MPU ổn định

    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, 0x75, 1, &check, 1, 10);
    if (check != 0x68) return;

    // Wake up, dùng PLL từ Gyro-X để có clock ổn định
    mData = 0x01;
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x6B, 1, &mData, 1, 10);
    HAL_Delay(10);

    // SMPLRT_DIV = 9 -> Sample Rate = 1kHz / (1 + 9) = 100Hz (Chu kỳ ngắt 10ms)
    mData = 0x09;
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x19, 1, &mData, 1, 10);

    // DLPF bandwidth ~44Hz - Lọc nhiễu rung động cơ bám sàn
    mData = 0x03;
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x1A, 1, &mData, 1, 10);

    // Gyro FS = ±250°/s -> Sensitivity 131 LSB/(°/s) - Độ phân giải cao nhất cho robot xoay
    mData = 0x00;
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x1B, 1, &mData, 1, 10);

    // Bật ngắt Data Ready (Thanh ghi 0x38), sửa timeout thành 10 như đã fix
    mData = 0x01;
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x38, 1, &mData, 1, 10);
}

// Ham hieu chuan: goi ham nay khi vua bat nguon
void mpu6050_Calibrate(void)
{
    long sumGZ = 0;
    // Đọc 2000 lần lấy trung bình nhiễu tĩnh trục Z
    for (int i = 0; i < 2000; i++) {
        uint8_t gy_data[2];
        // Chỉ đọc riêng 2 thanh ghi của trục Z (0x47 và 0x48) thay vì đọc cả 6 thanh ghi
        HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, 0x47, 1, gy_data, 2, 10);
        int16_t gz_raw = (int16_t)(gy_data[0] << 8 | gy_data[1]);
        sumGZ += gz_raw;
        HAL_Delay(1);
    }
    GZ_calib = (float)(sumGZ / 2000.0f) / 131.0f;
    yaw = 0.0f; // Reset góc robot về vị trí 0 ban đầu
}


// TỐI ƯU: Chỉ đọc đúng 2 byte dữ liệu vận tốc góc của trục Z (GYRO_ZOUT)
void mpu6050_readGyroZ(void)
{
    uint8_t gy_data[2];
    // Địa chỉ thanh ghi dữ liệu Trục Z của Gyro là 0x47
    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, 0x47, 1, gy_data, 2, 10);

    gz = (int16_t)(gy_data[0] << 8 | gy_data[1]);
    GZ = (float)gz / 131.0f;
}



// TỐI ƯU: Loại bỏ hoàn toàn bộ lọc bù phức tạp, chỉ tính tích phân Yaw tốc độ cao
void mpu6050_processYaw(float dt)
{
    // Trừ sai số tĩnh chống trôi góc
    float gz_corrected = GZ - GZ_calib;

    // Bộ lọc vùng chết (Dead-zone) tránh trôi góc khi robot đứng yên
    if (gz_corrected > -0.05f && gz_corrected < 0.05f) {
        gz_corrected = 0.0f;
    }

    // Tích phân Gyro Z theo thời gian thực để ra góc Yaw
    yaw += gz_corrected * dt;

    // Chuẩn hóa góc về khoảng [-180, 180] độ phục vụ thuật toán di chuyển
    if (yaw >  180.0f) yaw -= 360.0f;
    if (yaw < -180.0f) yaw += 360.0f;
}

/* ---------------------------------------------------------------
 * Tính góc lech ngan nhat gia 2 góc (xu lý wrap-around ±180°)
 * Ví du: angleDiff(170°, -170°) = -20°
 * --------------------------------------------------------------- */
/* Tính góc lệch ngắn nhất phục vụ thuật toán PID điều hướng hướng đi của robot */
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
        float dt = 0.01f; // Chu kỳ 10ms chuẩn xác từ phần cứng MPU

        mpu6050_readGyroZ();   // Chỉ đọc 2 byte trục Z qua I2C (Cực nhanh)
        mpu6050_processYaw(dt); // Cộng dồn góc xoay
        Flag++;
        if(Flag >= 2)
        {
        	Flag = 0;
        	static f32 d_l = 0.0f;
        	static f32 d_r = 0.0f;

        	f32 dl = ec_l.dist - d_l;
        	f32 dr = ec_r.dist - d_r;

        	d_l = ec_l.dist;
        	d_r = ec_r.dist;

        	Kinematics_update(dl, dr);
        	PH_task();

        	HeadingHold_Task();

        	motorcontrol_pid();
        }

    }
}

/*
 * mpu6050.c
 *
 *  Created on: Jun 7, 2026
 *      Author: GB Center
 */


#include "mpu6050.h"
#include "control.h"
#include "types.h"
#include "nav.h"
//Goi cau hinh i2c tu main.c


int16_t gz = 0;
float GZ = 0.0f;
float GZ_calib = 0.0f;
float   GZ_raw     = 0.0f;     /* gyro Z thô (chưa lọc) */
float   GZ_std     = 0.05f;    /* độ lệch chuẩn, fallback nếu calib chưa chạy */
float   GZ_deadzone= 0.05f;    /* dead-zone thích nghi, tính sau calibrate */

volatile float yaw = 0.0f;
#define GZ_LPF_ALPHA   0.35f

/* ── Bias re-tracking khi xe ĐỨNG YÊN — chống trôi góc dài hạn ──
 * τ rất lớn (factor nhỏ) → bias chỉ trôi theo VÀI CHỤC GIÂY, không
 * ảnh hưởng lúc đang di chuyển, chỉ âm thầm bù khi MPU nóng dần
 * lên làm offset gyro lệch nhẹ so với lúc mới calibrate.         */
#define BIAS_TRACK_RATE   0.0008f

static void _bias_retrack(void)
{
    u8 stationary = (ABS_F(ec_l.vel) < 0.01f) && (ABS_F(ec_r.vel) < 0.01f);
    if (!stationary) return;
    if (ABS_F(GZ - GZ_calib) > 2.0f*GZ_deadzone) return;

    GZ_calib += (GZ - GZ_calib) * BIAS_TRACK_RATE;
}

void mpu6050_Init(void)
{
    uint8_t check, mData;

    HAL_Delay(100);  // Chờ MPU ổn định

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 10);
    if (check != 0x68) return;

    // Wake up, dùng PLL từ Gyro-X để có clock ổn định
    mData = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &mData, 1, 10);
    HAL_Delay(10);

    // SMPLRT_DIV = 9 -> Sample Rate = 1kHz / (1 + 9) = 100Hz (Chu kỳ ngắt 10ms)
    mData = 0x09;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x19, 1, &mData, 1, 10);

    // DLPF bandwidth ~44Hz - Lọc nhiễu rung động cơ bám sàn
    mData = 0x03;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1A, 1, &mData, 1, 10);

    // Gyro FS = ±250°/s -> Sensitivity 131 LSB/(°/s) - Độ phân giải cao nhất cho robot xoay
    mData = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, 1, &mData, 1, 10);

    // Bật ngắt Data Ready (Thanh ghi 0x38), sửa timeout thành 10 như đã fix
    mData = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x38, 1, &mData, 1, 10);
}

// Ham hieu chuan: goi ham nay khi vua bat nguon
void mpu6050_Calibrate(void)
{


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
    /*
    long  sum  = 0;
    double sumsq = 0.0;
    uint8_t b[2];
 
    for (int i = 0; i < 2000; i++) {
        HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x47, 1, b, 2, 10);
        int16_t raw = (int16_t)((b[0]<<8)|b[1]);
        sum   += raw;
        sumsq += (double)raw * (double)raw;
        HAL_Delay(1);
    }
 
    float mean_raw = (float)sum / 2000.0f;
    GZ_calib = mean_raw / 131.0f;
 
    double var_raw = (sumsq/2000.0) - (double)mean_raw*(double)mean_raw;
    if (var_raw < 0.0) var_raw = 0.0;
    GZ_std = (float)(sqrt(var_raw) / 131.0);
    if (GZ_std < 0.015f) GZ_std = 0.015f;
 

    GZ_deadzone = limit(3.0f * GZ_std, 0.02f, 0.15f);
 
    yaw = 0.0f;
    GZ_raw = 0.0f;
    GZ     = 0.0f;
    */

}


// TỐI ƯU: Chỉ đọc đúng 2 byte dữ liệu vận tốc góc của trục Z (GYRO_ZOUT)
void mpu6050_readGyroZ(void)
{
    uint8_t gy_data[2];
    // Địa chỉ thanh ghi dữ liệu Trục Z của Gyro là 0x47
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x47, 1, gy_data, 2, 10);

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
    _bias_retrack();
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

    		if(ec_l.active && ((HAL_GetTick() - ec_l.tick) > 200))
    		{
    			ec_l.vel = 0;
    			ec_l.rpm = 0;
    			ec_l.active = 0;
    		}
    		if(ec_r.active && ((HAL_GetTick() - ec_r.tick) > 200))
    		{
    			ec_r.vel = 0;
    			ec_r.rpm = 0;
    			ec_r.active = 0;
    		}

    		 //HeadingHold_Task();
    		//turn_task();

    		if (turn.state == TR_IDLE || turn.state == TR_DONE || turn.state == TR_TOUT)
    		{
    		    HeadingHold_Task();
    		}
    		else
    		{
    		    turn_task();
    		}
    		//PH_task();


			//nav_task();
        	motorcontrol_pid();

        }

    }
}

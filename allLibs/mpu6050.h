#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"
#include "math.h"

// Define cac dia chi thanh ghi, toan hoc
#define     MPU6050_ADDR    0xD0				//dia chi i2c cua mpu6050 (dich phai 1 bit do i2c chi can 7 bit di chi)
#define     RTD             57.2957795f //quy tu goc Rad sang Deg

#define      MPU6050_INT_PORT    GPIOB
#define      MPU6050_PIN_INT     GPIO_PIN_12
extern I2C_HandleTypeDef hi2c1;

// khai bao extern de main.c co the doc dc debug truc tiep
extern volatile float yaw;
//Cac data ma mpu doc duoc
extern float GZ_calib;
// Funtion xu ly
void mpu6050_Init(void);
void mpu6050_Calibrate(void);
void mpu6050_readGyroZ(void); // Chi doc truc Z 
void mpu6050_processYaw(float dt);


float mpu6050_angleDiff(float target, float current);

#endif /* __MPU6050_H */

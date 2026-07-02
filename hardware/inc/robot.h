/*
 * robot.h
 *
 *  Created on: Jun 17, 2026
 *      Author: GB Center
 */

#ifndef INC_ROBOT_H_
#define INC_ROBOT_H_
#include "main.h"
#include "control.h"
#include "VL53L0X.h"
#include "nav.h"
#include "types.h"
#include "vl53_scan.h"
#include "avoid.h"
#include "stm32f4xx_hal.h"
#include "mpu6050.h"
#include "motor.h"
#include "OLED.h"
//#include "servo.h"
//#include "ssd1306.h"
//#include "delay.h"
//#include "OLED.h"

//#include "ssd136_fonts.h"
extern u32 t ;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart1;
void Robot_Init(void);
void Robot_Loop(void);


#endif /* INC_ROBOT_H_ */

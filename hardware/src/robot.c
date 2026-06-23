/*
 * robot.c
 *
 *  Created on: Jun 17, 2026
 *      Author: GB Center
 */
#include "robot.h"

void Robot_Init(void)
{
    /* LED sáng trong khi init */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

    //motor_init(7,1000);
    //encoder_init();
    //pid_setup();
    Kinematics_reset();
    /* VL53L0X */

    initVL53L0X(1, &hi2c2);
    //setTimeout(40);
    scanner_init();

    mpu6050_Init();
    mpu6050_Calibrate();
    hcsr04_init();

    //nav_init();
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    /* LED tắt → bắt đầu chạy */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

}
void Robot_Loop(void)
{
	//scanner_task();

}

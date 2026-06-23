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

<<<<<<< HEAD
//    motor_init(7,1000);
//    encoder_init();
//    pid_setup();
=======
    //motor_init(7,1000);
    //encoder_init();
    //pid_setup();
>>>>>>> dfba2d1e7edbc2e7e8239938aa3ea64ead7980dc
    Kinematics_reset();
    /* VL53L0X */

    initVL53L0X(1, &hi2c2);
    scanner_init();

    mpu6050_Init();
    mpu6050_Calibrate();
    hcsr04_init();

<<<<<<< HEAD
//    nav_init();
=======
    //nav_init();
>>>>>>> dfba2d1e7edbc2e7e8239938aa3ea64ead7980dc
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    /* LED tắt → bắt đầu chạy */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

}
void Robot_Loop(void)
{
<<<<<<< HEAD
//	scanner_task();
=======
	//scanner_task();
>>>>>>> dfba2d1e7edbc2e7e8239938aa3ea64ead7980dc

}

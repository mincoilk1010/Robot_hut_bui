/*
 * robot.c
 *
 *  Created on: Jun 17, 2026
 *      Author: GB Center
 */
#include "robot.h"
#include "main.h"
#include "string.h"
#include "stdio.h"
u32 t = 0;
extern I2C_HandleTypeDef hi2c3;
void Robot_Init(void)
{
    /* LED sáng trong khi init */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    motor_init(7,1000);
    encoder_init();
   // delay_init();
    pid_setup();
    Kinematics_reset();
    /* VL53L0X */

    if(initVL53L0X(1, &hi2c2) != 1) {
      char msg_err[] = "Khoi tao VL53L0X that bai\r\n";
      HAL_UART_Transmit(&huart1, (uint8_t*) msg_err, strlen(msg_err), 100);
      while(1); //Treo o day neu khoi tao that bai
    } else {
      char msg_ok[] = "Khoi tao VL53L0X thanh cong\r\n";
      HAL_UART_Transmit(&huart1, (uint8_t*) msg_ok, strlen(msg_ok), 100);
    }

    startContinuous(0);
    OLED_Init();
    ssd1306_SetCursor(10, 20);
    ssd1306_WriteString("ROBOT READY", Font_7x10, White);
    ssd1306_UpdateScreen();
    HAL_Delay(1000);

    mpu6050_Init();
    mpu6050_Calibrate();
	HeadingHold_SetTarget(yaw);    
	HeadingHold_Enable(1); 

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    scanner_init();

    nav_init();
    /* LED tắt → bắt đầu chạy */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
}
void Robot_Loop(void)
{
	scanner_task();

	static u32 t_nav = 0;
	u32 now = HAL_GetTick();

	/* Navigation owns state transitions and setpoints. The 20 ms MPU ISR
	 * remains the single owner of heading/turn and wheel PID execution. */
	if((u32)(now - t_nav) >= dt_ms){
		t_nav = now;
		nav_task();
	}

	/* Fallback/normal main-loop request. If MPU EXTI already executed this
	 * 20 ms slot, Control_Task20ms() returns without running PID twice. */
	Control_Task20ms();

	if((u32)(now - t) >= 150u){
		t = now;
		OLED_DrawRadarMap();
	}
	

}

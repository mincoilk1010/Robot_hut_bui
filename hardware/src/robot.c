/*
 * robot.c
 *
 *  Created on: Jun 17, 2026
 *      Author: GB Center
 */
#include "robot.h"
#include "main.h"
#include "string.h"
_vo u8  F = 0 ;
void Robot_Init(void)
{
    /* LED sáng trong khi init */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
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
    //OLED_Init();
    //ssd1306_SetCursor(10, 25);
    //ssd1306_WriteString("RADAR INIT...", Font_7x10, White);
    //ssd1306_UpdateScreen();

    mpu6050_Init();
    mpu6050_Calibrate();
    hcsr04_init();
    scanner_init();

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    nav_init();
    /* LED tắt → bắt đầu chạy */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    startContinuous(0);


    //ervo_Init(&htim3, TIM_CHANNEL_1);
    //Servo_WriteAngle(0);

}
void Robot_Loop(void)
{
	scanner_task();

	/* ── HC-SR04 đáy (cliff sensors): trigger định kỳ + chống treo ──
	 * Period 60ms = đủ cho echo về xa nhất (~4m, dùng cho timeout
	 * trong hcsr04_check_timeout) trước khi bắn trigger kế tiếp. */
	static u32 t_hc = 0;
	u32 now_hc = HAL_GetTick();
	hcsr04_check_timeout();
	if((u32)(now_hc - t_hc) >= 60u){
		t_hc = now_hc;
		hcsr04_read();
	}

	if(F)
	{
	    F = 0;

		nav_task();
	}
}

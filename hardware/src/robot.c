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
    hcsr04_init();

    /* Scanner controls TIM3 CH1 directly; Servo.c is intentionally unused. */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    scanner_init();

    nav_init();
    /* LED tắt → bắt đầu chạy */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
}
void Robot_Loop(void)
{
	scanner_task();

	static u32 t_hc = 0;
	static u32 t_nav = 0;
	static u32 t_debug = 0;
	static char debug_line[240];
	u32 now_hc = HAL_GetTick();
	hcsr04_check_timeout();
	/* hcsr04_read() alternates left/right, so 35 ms here gives each sensor
	 * a new sample about every 70 ms without acoustic cross-talk. */
	if((u32)(now_hc - t_hc) >= 35u){
		t_hc = now_hc;
		hcsr04_read();
	}

	/* Navigation owns state transitions and setpoints. The 20 ms MPU ISR
	 * remains the single owner of heading/turn and wheel PID execution. */
	if((u32)(now_hc - t_nav) >= dt_ms){
		t_nav = now_hc;
		nav_task();
	}

	/* Fallback/normal main-loop request. If MPU EXTI already executed this
	 * 20 ms slot, Control_Task20ms() returns without running PID twice. */
	Control_Task20ms();

	if((u32)(now_hc - t_debug) >= 500u){
		t_debug = now_hc;
		int n = snprintf(debug_line, sizeof(debug_line),
			"DBG t=%lu nav=%u v=%.3f w=%.3f front=%u fseq=%lu "
			"sm=%u wr=%u hcL=%lu hcR=%lu sl=%.3f sr=%.3f "
			"pl=%d pr=%d c1=%lu c2=%lu c3=%lu c4=%lu turn=%u\r\n",
			(unsigned long)now_hc, (unsigned)nav.st,
			(double)g_sp_v, (double)g_sp_w,
			(unsigned)scanner_front(), (unsigned long)sc.front_seq,
			(unsigned)sc.mode, (unsigned)sc.wide_ready,
			(unsigned long)hc[HC_LEFT_INDEX].d,
			(unsigned long)hc[HC_RIGHT_INDEX].d,
			(double)sl, (double)sr, (int)p_l, (int)p_r,
			(unsigned long)TIM4->CCR1, (unsigned long)TIM4->CCR2,
			(unsigned long)TIM4->CCR3, (unsigned long)TIM4->CCR4,
			(unsigned)turn.state);
		if(n > 0){
			if(n >= (int)sizeof(debug_line)) n = (int)sizeof(debug_line) - 1;
			HAL_UART_Transmit(&huart1, (u8*)debug_line, (u16)n, 30u);
		}
	}
	
	if((u32)(now_hc - t) >= 150u){
		t = now_hc;
		OLED_DrawRadarMap();
	}
	


}

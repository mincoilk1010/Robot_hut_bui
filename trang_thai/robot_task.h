/*
 * robot_task.h
 * Nơi chứa máy trạng thái điều hướng dọn dẹp tổng thể
 */

#ifndef ROBOT_TASK_H_
#define ROBOT_TASK_H_

#include "main.h"

// Gọi 1 lần duy nhất trước khi vào while(1)
void Robot_Task_Init(void);

// Gọi liên tục trong while(1)
void Robot_State_Machine(void);

#endif /* ROBOT_TASK_H_ */

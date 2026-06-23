/*
 * vl53_scan.h
 *
 *  Created on: Jun 17, 2026
 *      Author: GB Center
 */

#ifndef INC_VL53_SCAN_H_
#define INC_VL53_SCAN_H_

#include "types.h"
#include "VL53L0X.h"
#include "main.h"
#include "types.h"
#define SC_OBS_MM    400u   /* < 40cm → wide mode */
#define SC_CLEAR_MM  600u   /* > 60cm × 5 → narrow mode */
#define SC_N_MIN      60u
#define SC_N_MAX     120u
#define SC_W_MIN       0u
#define SC_W_MAX     180u
#define SC_STEP        5u
#define SC_WAIT_MS    22u   /* servo settle */

typedef enum{
    SC_IDLE,
    SC_MOVE,
    SC_WAIT,
    SC_READ
}ScSt_t;
typedef enum{
    SC_NARROW=0,
    SC_WIDE=1
}ScMode_t;
typedef struct{
    ScMode_t mode; ScSt_t state;
    u8 angle,amin,amax;
    u32 t_servo;
    i8 dir;
    u16 data[37];          /* data[deg/5] */
    u8 done; u32 cycle;
    u8 clear_cnt;
} Scanner_t;
extern Scanner_t sc;

extern TIM_HandleTypeDef htim3;
extern u16 d;
void _svo(u8 deg);
void    scanner_init(void);
void    scanner_task(void);
u16     scanner_get(u8 deg);
u16     scanner_front(void);
u8      scanner_has_obs(void);
ScMode_t scanner_mode(void);
u8 scanner_wide_ready(void);
#endif /* INC_VL53_SCAN_H_ */

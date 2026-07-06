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
#define SC_OBS_MM    500u   /* < 50cm: start wide scan early for planning */
#define SC_CLEAR_MM  500u   /* > 50cm x 5 -> narrow mode */
#define SC_N_MIN      60u
#define SC_N_MAX     120u
#define SC_W_MIN       0u
#define SC_W_MAX     180u
#define SC_STEP        5u
#define SC_WAIT_MS    18u   /* servo settle */
#define SC_SAMPLE_MAX_AGE_MS       1200u
#define SC_FRONT_NARROW_MAX_AGE_MS  700u
#define SC_FRONT_WIDE_MAX_AGE_MS   1200u
#define SC_LOCK_MAX_AGE_MS          200u
#define SC_MAP_BINS                 ((SC_W_MAX - SC_W_MIN) / SC_STEP + 1u)
#define SC_MAP_MAX_AGE_MS           1200u
#define SC_MAP_CONF_MAX             10u

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
    u32 stamp[37];         /* time each direction was measured */
    u8 wide_valid[37];     /* measured since entering wide mode */
    u8 wide_valid_count;
    u8 wide_ready;         /* latched until navigation consumes the scan */
    u8 wide_hold;          /* navigation owns this wide-scan result */
    u8 front_mask;         /* bits for fresh 85/90/95 degree samples */
    u16 front_mm;          /* median of the last complete front frame */
    u32 front_stamp;
    u32 front_seq;         /* increments once per complete front frame */
    u8 locked;             /* 1 = servo locked to lock_angle */
    u8 lock_angle;
    u16 lock_mm;
    u32 lock_stamp;
    u32 lock_seq;
    u8 done; u32 cycle;
    u8 clear_cnt;
} Scanner_t;

typedef struct {
    u16 raw_mm;       /* Last raw sample at this angle */
    u16 filt_mm;      /* Filtered sample for display */
    u32 stamp;        /* Last good-sample time */
    u8 valid;         /* 1 = usable for display */
    u8 conf;          /* confidence 0..SC_MAP_CONF_MAX */
} Vl53MapCell_t;

extern Scanner_t sc;
extern Vl53MapCell_t vl53_map[SC_MAP_BINS];

extern TIM_HandleTypeDef htim3;
extern u16 d;
void    scanner_init(void);
void    scanner_task(void);
u16     scanner_get(u8 deg);
u16     scanner_front(void);
u8      scanner_has_obs(void);
ScMode_t scanner_mode(void);
u8 scanner_wide_ready(void);
void scanner_request_wide(void);
void scanner_wide_consume(void);
u32 scanner_front_seq(void);
void scanner_lock_angle(u8 deg);
void scanner_unlock(void);
u16 scanner_locked_mm(void);
u32 scanner_locked_seq(void);
void scanner_map_clear(void);
u16 scanner_map_get(u8 deg);
u8 scanner_map_conf(u8 deg);
#endif /* INC_VL53_SCAN_H_ */

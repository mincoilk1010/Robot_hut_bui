/*
 * types.h
 *
 *  Created on: Jun 7, 2026
 *      Author: GB Center
 */

#ifndef INC_TYPES_H_
#define INC_TYPES_H_
#include <stdint.h>
#include "stddef.h"
#define _vo   volatile
typedef float f32;
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
#define WHEEL_DIAMETER_M    0.065f          /* đường kính bánh */
#define WHEEL_RADIUS_M      (WHEEL_DIAMETER_M * 0.5f)
#define WHEEL_BASE_M        0.15f          /* khoảng cách 2 bánh */
#define ENCODER_PPR         20              /* xung/vòng (1 kênh) */
#define PI                3.14159265f


#define dt_s                0.02f
#define dt_ms               20u


#define KP_r 1330.0f
#define KI_r 450.0f
#define KD_r 0.03f
#define KP_l 1430.0f
#define KI_l 450.0f
#define KD_l 0.03f
#define pid_int_min       -999.0f
#define pid_int_max          999.0f
#define pid_out_min        -999.0f
#define pid_out_max        999.0f


#define DEG2RAD(x)          ((x) * PI / 180.0f)
#define RAD2DEG(x)          ((x) * 180.0f / PI)
#define limit(x,lo,hi)      ((x)<(lo)?(lo):((x)>(hi)?(hi):(x)))
#define ABS_F(x)            ((x)<0.0f?-(x):(x))



#endif /* INC_TYPES_H_ */

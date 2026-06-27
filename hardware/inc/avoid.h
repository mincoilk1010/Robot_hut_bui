/*
 * avoid.h
 *
 *  Created on: Jun 17, 2026
 *      Author: GB Center
 */

#ifndef INC_AVOID_H_
#define INC_AVOID_H_

#include "types.h"
 

#define ROBOT_WIDTH_M     0.25f   /* chiều rộng thân xe: 25 cm */
#define SAFETY_MARGIN_M   0.17f   /* tổng biên an toàn trái + phải */
#define MIN_GAP_M         (ROBOT_WIDTH_M + SAFETY_MARGIN_M) /* 0.40 m */
 
/* Ngưỡng coi là "thoáng" khi tìm gap (mm) */
#define GAP_CLEAR_MM      450u
#define GAP_UNKNOWN_MM    1500u
#define GAP_UNKNOWN_PENALTY_MM 40.0f
#define GAP_UNKNOWN_MIN_SPAN_POINTS 5u
#define GAP_REAL_MIN_POINTS 2u
#define GAP_UNKNOWN_MAX_RATIO 2u
 
typedef struct {
    uint8_t  found;        /* 1 nếu tìm được khe hở hợp lệ */
    uint8_t  center_deg;   /* góc giữa khe hở (servo angle 0-180) */
    float    width_m;      /* chiều rộng khe hở ước tính (m) */
    uint16_t avg_mm;        /* khoảng cách trung bình trong khe (mm) */
    float    redirect_deg; /* = center_deg - 90 → góc cần xoay thêm */
} Gap_t;
 
/* Quét sc.data[] (đã có sau 1 vòng wide scan) tìm khe hở tốt nhất.
 * Trả 1 nếu tìm được khe đủ rộng cho xe đi qua, 0 nếu bị chặn hết. */
uint8_t Avoid_FindBestGap(Gap_t *out);

#endif /* INC_AVOID_H_ */

/*
 * avoid.h
 *
 *  Created on: Jun 17, 2026
 *      Author: GB Center
 */

#ifndef INC_AVOID_H_
#define INC_AVOID_H_

#include "types.h"
 

#define ROBOT_WIDTH_M     0.20f   /* chiều rộng thân xe */
#define SAFETY_MARGIN_M   0.10f   /* biên an toàn 2 bên */
#define MIN_GAP_M         (ROBOT_WIDTH_M + SAFETY_MARGIN_M) /* 0.30m */
 
/* Ngưỡng coi là "thoáng" khi tìm gap (mm) */
#define GAP_CLEAR_MM      450u
 
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

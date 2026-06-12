
#ifndef INC_NAVIGATION_H_
#define INC_NAVIGATION_H_

#include <stdint.h>

// Định nghĩa các ngưỡng khoảng cách để quyết định hướng di chuyển (THRESHOLDS)
#define SAFE_DISTANCE 200        // Khoảng cách an toàn để di chuyển thẳng (20cm)
#define EDGE_JUMP_THRESHOLD 150  // Khoảng cách bước nhảy để phát hiện cạnh (15cm)

// Khai báo các hướng quyết định
typedef enum {
    DIR_FORWARD,
    DIR_LEFT,
    DIR_RIGHT,
    DIR_U_TURN
} Nav_Direction;

// Prototype hàm ra quyết định lách vật cản
Nav_Direction Scan_and_Decide(void);

#endif /* INC_NAVIGATION_H_ */
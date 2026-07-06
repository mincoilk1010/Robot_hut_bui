#ifndef OLED_H
#define OLED_H

#include <stdint.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "vl53_scan.h"

// Cau hinh ty le hien thi
#define MAX_RADAR_DIST_MM  1000u
#define RADAR_RADIUS_X_PIXEL ((float)(SSD1306_WIDTH / 2u - 5u))
#define RADAR_RADIUS_Y_PIXEL ((float)(SSD1306_HEIGHT - 8u))
#define OLED_RADAR_STEP_DEG 10u

#define OLED_ORIGIN_X (SSD1306_WIDTH / 2u)
#define OLED_ORIGIN_Y (SSD1306_HEIGHT - 1u)

// Khai bao ham 
void OLED_Init(void);
void OLED_DrawRadarMap(void);

#endif // OLED_H

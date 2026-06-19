#ifndef OLED_H
#define OLED_H

#include <stdint.h>
#include "ssd1306.h"
#include "navigation.h" // De lay mang Lidar_Map

// Cau hinh ty le hien thi
#define MAX_RADAR_DIST_MM  1000.0f  // Khoang cach toi da muon hien thi tren radar (1000mm)
#define RADAR_RADIUS_PIXEL 60.0f    // Ban kinh radar tren man hinh (chua lai 4 pixel cho le)

#define OLED_ORIGIN_X 64 // Tam xe nam o giua chieu ngang (128/2)
#define OLED_ORIGIN_Y 63 // Tam xe nam o sat day man hinh (64-1)

// Khai bao ham 
void OLED_Init(void);
void OLED_DrawRadarMap(void);

#endif // OLED_H
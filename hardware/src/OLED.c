#include "OLED.h"

#include <math.h>
#include <stdio.h>

#define OLED_PI 3.14159265f

void OLED_Init(void)
{
    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_UpdateScreen();
}

void OLED_DrawRadarMap(void)
{
    ssd1306_Fill(Black);

    /* Radar grid: old style, but scaled to the configured screen size. */
    ssd1306_DrawCircle(OLED_ORIGIN_X, OLED_ORIGIN_Y, 2, White);
    ssd1306_Line(OLED_ORIGIN_X, OLED_ORIGIN_Y,
                 OLED_ORIGIN_X - (uint8_t)(RADAR_RADIUS_X_PIXEL * 0.70f),
                 OLED_ORIGIN_Y - (uint8_t)(RADAR_RADIUS_Y_PIXEL * 0.70f),
                 White);
    ssd1306_Line(OLED_ORIGIN_X, OLED_ORIGIN_Y,
                 OLED_ORIGIN_X, 3, White);
    ssd1306_Line(OLED_ORIGIN_X, OLED_ORIGIN_Y,
                 OLED_ORIGIN_X + (uint8_t)(RADAR_RADIUS_X_PIXEL * 0.70f),
                 OLED_ORIGIN_Y - (uint8_t)(RADAR_RADIUS_Y_PIXEL * 0.70f),
                 White);

    int prev_x = -1;
    int prev_y = -1;

    uint8_t map_points = 0u;

    /* Draw a sparse contour from real VL53 returns only.
     * Unknown / too-far directions are skipped, so the OLED stays clean.
     */
    for (int angle = SC_W_MIN; angle <= SC_W_MAX; angle += OLED_RADAR_STEP_DEG) {
        uint16_t dist = scanner_map_get((uint8_t)angle);

        if (dist == 0u || dist == 9999u || dist > MAX_RADAR_DIST_MM) {
            prev_x = -1;
            prev_y = -1;
            continue;
        }
        map_points++;

        float k = (float)dist / (float)MAX_RADAR_DIST_MM;
        float rad = (float)angle * OLED_PI / 180.0f;
        int x = OLED_ORIGIN_X + (int)(k * RADAR_RADIUS_X_PIXEL * cosf(rad));
        int y = OLED_ORIGIN_Y - (int)(k * RADAR_RADIUS_Y_PIXEL * sinf(rad));

        if (x < 0) x = 0;
        if (x >= SSD1306_WIDTH) x = SSD1306_WIDTH - 1;
        if (y < 0) y = 0;
        if (y >= SSD1306_HEIGHT) y = SSD1306_HEIGHT - 1;

        if (prev_x >= 0 && prev_y >= 0) {
            ssd1306_Line(prev_x, prev_y, x, y, White);
        }

        ssd1306_DrawCircle((uint8_t)x, (uint8_t)y, 1, White);
        prev_x = x;
        prev_y = y;
    }

    char text[20];
    uint16_t front = scanner_front();
    if (front == 9999u) snprintf(text, sizeof(text), "F:--- M:%u", map_points);
    else                snprintf(text, sizeof(text), "F:%u M:%u", front, map_points);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString(text, Font_7x10, White);

    ssd1306_UpdateScreen();
}

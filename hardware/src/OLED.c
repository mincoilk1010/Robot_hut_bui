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

    ssd1306_DrawCircle(OLED_ORIGIN_X, OLED_ORIGIN_Y, 2, White);
    ssd1306_Line(OLED_ORIGIN_X, OLED_ORIGIN_Y,
                 OLED_ORIGIN_X - 42, OLED_ORIGIN_Y - 42, White);
    ssd1306_Line(OLED_ORIGIN_X, OLED_ORIGIN_Y,
                 OLED_ORIGIN_X, 3, White);
    ssd1306_Line(OLED_ORIGIN_X, OLED_ORIGIN_Y,
                 OLED_ORIGIN_X + 42, OLED_ORIGIN_Y - 42, White);

    int prev_x = -1;
    int prev_y = -1;

    /* Draw scanner bins directly; no occupancy map is required. */
    for (int angle = SC_W_MIN; angle <= SC_W_MAX; angle += SC_STEP) {
        uint16_t dist = sc.data[angle / SC_STEP];
        if (dist == 0u || dist > MAX_RADAR_DIST_MM) {
            prev_x = -1;
            prev_y = -1;
            continue;
        }

        float r_pixel = ((float)dist / (float)MAX_RADAR_DIST_MM) *
                        RADAR_RADIUS_PIXEL;
        float rad = (float)angle * OLED_PI / 180.0f;
        int x = OLED_ORIGIN_X + (int)(r_pixel * cosf(rad));
        int y = OLED_ORIGIN_Y - (int)(r_pixel * sinf(rad));

        if (x < 0) x = 0;
        if (x > 127) x = 127;
        if (y < 0) y = 0;
        if (y > 63) y = 63;

        if (prev_x >= 0 && prev_y >= 0) {
            ssd1306_Line(prev_x, prev_y, x, y, White);
        }
        ssd1306_DrawPixel(x, y, White);
        prev_x = x;
        prev_y = y;
    }

    char text[20];
    uint16_t front = scanner_front();
    if (front == 9999u) snprintf(text, sizeof(text), "F:---");
    else                snprintf(text, sizeof(text), "F:%umm", front);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString(text, Font_7x10, White);

    ssd1306_UpdateScreen();
}

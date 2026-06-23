/*
 * map.c
 *
 *  Created on: Jun 15, 2026
 *      Author: GB Center
 */
#include "map.h"
#include "string.h"
#include "math.h"
#include <stdlib.h>
#include "ssd1306.h"
#include "control.h"
i8 grid[MAP_H_G][MAP_W];
uint16_t Lidar_Map[181];
uint16_t MAX_RADAR_DIST_MM;
void map_init()
{
    memset(grid, 0, sizeof(grid));
}


static void _ray_free(int x0, int y0, int x1, int y1)
{
    int dx = abs(x1-x0), dy = abs(y1 - y0);
    int sx = x0<x1?1:-1;
    int sy = y0<y1?1:-1;
    int e = dx - dy;
    int e2;
    for(;;)
    {
        if(_gv(x0,y0)) grid[y0][x0] = limit((i16)grid[y0][x0] + 12,-100,100);
        if(x0==x1 && y0==y1) break;
        e2 = 2*e;
        if(e2>-dy)
        {
            e -= dy;
            x0 += sx;
        }
        if(e2<dx)
        {
            e += dx;
            y0 += sy;
        }
    }
    
}
void map_update(f32 rx, f32 ry,f32 rth, f32 sdeg, u16 dmm)
{
    f32 a = rth + DEG2RAD(sdeg - 90);
    int rox = _gx(rx), roy = _gy(ry);
    if(dmm > 2000 || dmm == 9999)
    {
        _ray_free(rox,roy,_gx(rx+2.0f*cosf(a)),_gy(ry+2.0f*sinf(a)));
        return;
    }
    f32 dm = (f32)dmm/1000.0f;
    _ray_free(rox,roy,_gx(rx+((dm-MAP_CELL)*cosf(a))),_gy(ry+(dm-MAP_CELL)*sinf(a)));
    int ogx = _gx(rx+dm*cosf(a)), ogy = _gy(ry + dm * sinf(a));
    for(int dy=-1;dy<=1;dy++)
    {
        for(int dx=-1; dx<=1;dx++)
        {
            int nx = ogx + dx, ny = ogy+dy;
            if(_gv(nx,ny)) grid[ny][nx] = limit((i16)grid[ny][nx] - 25,-100,100);
        }
    }

}
void map_mark_robot(f32 rx, f32 ry)
{
    int gx = _gx(rx), gy = _gy(ry);
    for(int dy=-1;dy<=1;dy++)
    {
        for(int dx=-1;dx<=1;dx++)
        {
            int nx = gx + dx, ny = gy + dy;
            if(_gv(nx,ny) && grid[ny][nx] >= 0)
            {
                grid[ny][nx] = limit((i16)grid[ny][nx]+12,-100,100);
            }
        }
    }

}
u8 map_obs(f32 wx, f32 wy)
{
    int gx = _gx(wx) , gy = _gy(wy);
    return _gv(gx,gy) && grid[gy][gx] < -10;
}
#define OLED_MAP_OX   ((SSD1306_WIDTH  - MAP_W)    / 2)
#define OLED_MAP_OY   ((SSD1306_HEIGHT - MAP_H_G)  / 2)

void map_draw_oled(void)
{
    ssd1306_Fill(Black);

    /* ===== GRID ===== */
    for (int gy = 0; gy < MAP_H_G; gy++)
    for (int gx = 0; gx < MAP_W; gx++)
    {
        if (grid[gy][gx] < -10)
        {
            uint8_t px = OLED_MAP_OX + gx;
            uint8_t py = OLED_MAP_OY + (MAP_H_G - 1 - gy);
            ssd1306_DrawPixel(px, py, White);
        }
    }

    /* ===== ROBOT POSITION ===== */
    int rgx = _gx(pose.x);
    int rgy = _gy(pose.y);

    if (!_gv(rgx, rgy))
    {
        ssd1306_UpdateScreen();
        return;
    }

    uint8_t rpx = OLED_MAP_OX + rgx;
    uint8_t rpy = OLED_MAP_OY + (MAP_H_G - 1 - rgy);

    /* robot 3x3 */
    for (int dy = -1; dy <= 1; dy++)
    for (int dx = -1; dx <= 1; dx++)
        ssd1306_DrawPixel(rpx + dx, rpy + dy, White);

    /* heading */
    float a = pose.theta;

    int hx = rpx + (int)(cosf(a) * 6);
    int hy = rpy - (int)(sinf(a) * 6);

    ssd1306_Line(rpx, rpy, hx, hy, White);

    /* ===== RADAR POINT CLOUD ===== */
    for (int angle = 0; angle <= 180; angle += 10)
    {
        uint16_t dist = Lidar_Map[angle];

        if (dist == 0 || dist > MAX_RADAR_DIST_MM)
            continue;

        float r = ((float)dist / MAX_RADAR_DIST_MM) * 6.0f;
        float rad = angle * 3.1415926f / 180.0f;

        int x = rpx + (int)(r * cosf(rad));
        int y = rpy - (int)(r * sinf(rad));

        if (_gv(x, y))
            ssd1306_DrawPixel(x, y, White);
    }

    ssd1306_UpdateScreen();
}

/*
 * map.h
 *
 *  Created on: Jun 15, 2026
 *      Author: GB Center
 */

#ifndef INC_MAP_H_
#define INC_MAP_H_

#include "types.h"

#define MAP_W 60
#define MAP_H_G 60
#define MAP_CELL 2.0f 
#define MAP_OX 30
#define MAP_OY 30

extern i8 grid[MAP_H_G][MAP_W];
extern uint16_t Lidar_Map[181];
extern uint16_t MAX_RADAR_DIST_MM;

void map_init();
void map_update(f32 rx, f32 ry,f32 rth, f32 sdeg, u16 dmm);
void map_mark_robot(f32 rx, f32 ry);
u8 map_obs(f32 wx, f32 wy);
static inline int _gx(f32 w){return (int)(w / MAP_CELL) + MAP_OX;}
static inline int _gy(f32 w){return (int)(w / MAP_CELL) + MAP_OY;}
static inline int _gv(int x, int y){return x>= 0 && x<MAP_W && y>=0 && y<MAP_H_G;}
void map_draw_oled(void);


#endif /* INC_MAP_H_ */

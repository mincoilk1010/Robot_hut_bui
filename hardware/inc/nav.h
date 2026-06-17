/*
 * nav.h
 *
 *  Created on: Jun 17, 2026
 *      Author: GB Center
 */

#ifndef INC_NAV_H_
#define INC_NAV_H_
#include "types.h"

#define NAV_SPEED    0.15f
#define NAV_ROW_M    0.30f
#define NAV_MAX_ROWS 10
#define NAV_OBS_MM   270u
typedef enum{
    N_BOOT,N_FWD,N_BRAKE,N_TURN90,
    N_CROSS,N_TURN90B,N_CLIFF,N_DONE
}NavSt_t;
typedef struct{
    NavSt_t st; i8 dir; u8 row;
    float lane_yaw,x0,y0;
    u32 t0; u8 done;
}Nav_t;
extern Nav_t nav;
void nav_init(void);
void nav_task(void);


#endif /* INC_NAV_H_ */

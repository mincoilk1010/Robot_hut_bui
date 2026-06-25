#ifndef INC_NAV_H_
#define INC_NAV_H_

#include "types.h"
#include "main.h"

#define NAV_SPEED              0.12f
#define NAV_ROW_M              0.30f
#define NAV_WALL_ROW_M         0.10f
#define NAV_MAX_ROWS           10u
#define NAV_OBS_SLOW_MM        450u
#define NAV_OBS_TURN_MM        250u
#define NAV_OBS_CONTACT_MM     160u
#define NAV_ROW_LENGTH_M       2.80f
#define NAV_ACCEL_MPS2         0.30f
#define NAV_STOP_SETTLE_MS    100u
#define NAV_SCAN_TIMEOUT_MS   4000u

#define NAV_XTRACK_K           1.20f
#define NAV_XTRACK_W_MAX       0.12f

#define NAV_AVOID_ANGLE_DEG    90.0f
#define NAV_AVOID_OFFSET_M     0.40f
#define NAV_AVOID_PASS_M       0.80f
#define NAV_AVOID_REJOIN_M     0.60f
#define NAV_AVOID_REJOIN_LEAD_M 0.04f

#define NAV_SCAN_SEGMENT_JUMP_MM 200.0f
#define NAV_WALL_MIN_POINTS       8u
#define NAV_WALL_MIN_LENGTH_M     0.55f
#define NAV_WALL_LINE_RMS_MAX_M   0.08f
#define NAV_WALL_SPAN_LEFT_DEG   65.0f
#define NAV_WALL_SPAN_RIGHT_DEG 115.0f
#define NAV_WALL_END_AXIS_MAX     0.75f

#define NAV_STUCK_ENABLE          1u
#define NAV_STUCK_CMD_MIN_MPS     0.05f
#define NAV_STUCK_VEL_MAX_MPS     0.015f
#define NAV_STUCK_PROGRESS_MIN_M  0.015f
#define NAV_STUCK_TIME_MS         900u
#define NAV_STUCK_PWM_MIN         600
#define NAV_HIDDEN_ROW_PWM_MIN    600
#define NAV_STUCK_BACK_M          0.20f
#define NAV_STUCK_BACK_SPEED      0.06f
#define NAV_STUCK_BACK_TIMEOUT_MS 4500u
#define NAV_ROW_CROSS_STUCK_GRACE_MS 1800u
#define NAV_BLOCKED_NEAR_MM       250u
#define NAV_BLOCKED_NEAR_POINTS     8u
#define NAV_BLOCKED_FRONT_POINTS    2u
#define NAV_SEARCH_STEP_DEG       30.0f
#define NAV_SEARCH_MAX_STEPS         4u

#define NAV_TURN_RETRY_MAX       1u
#define NAV_TURN_ACCEPT_ERR_DEG  8.0f
#define NAV_TURN_COARSE_ACCEPT_ERR_DEG 30.0f
#define NAV_ROW_TURN_RETRY_MAX   2u
#define NAV_ROW_TURN_ACCEPT_ERR_DEG 45.0f

#define NAV_CLIFF_BACK_M       0.10f
#define NAV_CLIFF_ESCAPE_M     0.18f
#define NAV_CLIFF_SPEED        0.06f
#define NAV_CLIFF_ENABLE       0u   /* HC-SR04 is not connected yet */

/* Swap these two values if the physical HC-SR04 wiring is reversed. */
#define HC_LEFT_INDEX          0u
#define HC_RIGHT_INDEX         1u

typedef enum {
    N_BOOT = 0,
    N_FWD,
    N_BRAKE,
    N_TURN_WAIT,
    N_TURN_ACTIVE,
    N_ROW_CROSS,
    N_AVOID_OFFSET,
    N_AVOID_PASS,
    N_AVOID_REJOIN,
    N_CLIFF_CONFIRM,
    N_CLIFF_BACK,
    N_CLIFF_DECIDE,
    N_CLIFF_SCAN,
    N_CLIFF_ESCAPE,
    N_DONE,
    N_STUCK_BACK
} NavSt_t;

typedef struct {
    NavSt_t st;
    i8 dir;                 /* +1 = left, -1 = right */
    u8 row;
    float lane_yaw;
    float x0;
    float y0;
    u32 t0;
    u8 done;
} Nav_t;

extern Nav_t nav;
extern UART_HandleTypeDef huart1;

void nav_init(void);
void nav_task(void);

#endif /* INC_NAV_H_ */

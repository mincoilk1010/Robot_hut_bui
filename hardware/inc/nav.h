#ifndef INC_NAV_H_
#define INC_NAV_H_

#include "types.h"
#include "main.h"

#define NAV_SPEED              0.14f
#define NAV_ROW_M              0.30f
#define NAV_WALL_ROW_M         0.30f
#define NAV_MAX_ROWS           10u
#define NAV_OBS_LOOKAHEAD_MM   600u
#define NAV_OBS_DECIDE_MM      350u
#define NAV_OBS_SLOW_MM        600u
#define NAV_OBS_TURN_MM        150u
#define NAV_OBS_CONTACT_MM     160u
#define NAV_ROW_LENGTH_M       2.80f
#define NAV_ACCEL_MPS2         0.40f
#define NAV_BOOT_DELAY_MS     300u
#define NAV_STOP_SETTLE_MS     80u
#define NAV_BRAKE_MS           50u
#define NAV_SCAN_TIMEOUT_MS    2400u
#define NAV_SCAN_MIN_VALID_POINTS 10u
#define NAV_LOOKAHEAD_RESCAN_MS 500u
#define NAV_CLASS_CONFIRM_COUNT  1u

#define NAV_XTRACK_K           0.90f
#define NAV_XTRACK_W_MAX       0.10f
#define NAV_XTRACK_DEADBAND_M  0.015f

#define NAV_AVOID_ANGLE_DEG    90.0f
#define NAV_AVOID_MIN_ANGLE_DEG 45.0f
#define NAV_AVOID_SPEED        0.12f
#define NAV_AVOID_OFFSET_M     0.40f
#define NAV_AVOID_PASS_M       0.80f
#define NAV_AVOID_REJOIN_M     0.60f
#define NAV_AVOID_REJOIN_LEAD_M 0.04f
#define NAV_AVOID_FRONT_IGNORE_M 0.22f
#define NAV_AVOID_FRONT_IGNORE_MS 1000u
#define NAV_ROBOT_LENGTH_M     0.35f
#define NAV_AVOID_SIDE_SAFE_M  0.08f
#define NAV_AVOID_PASS_SAFE_M  0.15f
#define NAV_AVOID_OFFSET_MIN_M 0.35f
#define NAV_AVOID_OFFSET_MAX_M 0.65f
#define NAV_AVOID_PASS_MIN_M   0.75f
#define NAV_AVOID_PASS_MAX_M   1.30f
#define NAV_OBJECT_MIN_WIDTH_M 0.08f
#define NAV_OBJECT_MAX_WIDTH_M 0.80f
#define NAV_SIDE_LOOK_LEFT_DEG   180u
#define NAV_SIDE_LOOK_RIGHT_DEG    0u
#define NAV_SIDE_OBJECT_MM       700u
#define NAV_SIDE_LOST_COUNT        4u
#define NAV_AVOID_SIDE_MIN_M    0.35f
#define NAV_PASS_NO_OBJECT_M    0.35f
#define NAV_AFTER_OBJECT_CLEAR_M 0.35f
#define NAV_AVOID_NO_SIDE_OFFSET_M 0.20f

#define NAV_FOLLOW_TARGET_MM     250u
#define NAV_FOLLOW_TARGET_TOL_MM  60u
#define NAV_FOLLOW_LOST_MM       650u
#define NAV_FOLLOW_FRONT_CLEAR_MM 450u
#define NAV_FOLLOW_LOST_COUNT      3u
#define NAV_FOLLOW_CLEAR_M      0.25f
#define NAV_FOLLOW_MIN_M        0.25f
#define NAV_FOLLOW_SPEED        0.10f
#define NAV_FOLLOW_K            1.05f
#define NAV_FOLLOW_W_MAX        0.28f
#define NAV_FOLLOW_FILTER_ALPHA 0.35f

#define NAV_SCAN_SEGMENT_JUMP_MM 200.0f
#define NAV_WALL_MIN_POINTS       8u
#define NAV_WALL_NEAR_EXTRA_MM  120u
#define NAV_WALL_BROAD_SPAN_DEG  70u
#define NAV_WALL_MID_SPAN_DEG    55u
#define NAV_WALL_MIN_LENGTH_M     0.55f
#define NAV_WALL_LINE_RMS_MAX_M   0.08f
#define NAV_WALL_SPAN_LEFT_DEG   65.0f
#define NAV_WALL_SPAN_RIGHT_DEG 115.0f
#define NAV_WALL_END_AXIS_MAX     0.75f

#define NAV_STUCK_ENABLE          1u
#define NAV_STUCK_CMD_MIN_MPS     0.05f
#define NAV_STUCK_VEL_MAX_MPS     0.015f
#define NAV_STUCK_PROGRESS_MIN_M  0.015f
#define NAV_STUCK_TIME_MS         450u
#define NAV_STUCK_PWM_MIN         600
#define NAV_HIDDEN_ROW_PWM_MIN    600
#define NAV_STUCK_BACK_M          0.20f
#define NAV_STUCK_BACK_SPEED      0.06f
#define NAV_STUCK_BACK_TIMEOUT_MS 4500u
#define NAV_ROW_CROSS_STUCK_GRACE_MS 1300u
#define NAV_BLOCKED_NEAR_MM       250u
#define NAV_BLOCKED_NEAR_POINTS     8u
#define NAV_BLOCKED_FRONT_POINTS    2u
#define NAV_SEARCH_STEP_DEG       30.0f
#define NAV_SEARCH_MAX_STEPS         4u

#define NAV_TURN_RETRY_MAX       1u
#define NAV_TURN_ACCEPT_ERR_DEG  4.0f
#define NAV_TURN_COARSE_ACCEPT_ERR_DEG 8.0f
#define NAV_AVOID_TURN_ACCEPT_ERR_DEG 6.0f
#define NAV_AVOID_TURN_FORCE_MS 2200u
#define NAV_AVOID_TURN_FORCE_ERR_DEG 12.0f
#define NAV_ROW_TURN_RETRY_MAX   3u
#define NAV_ROW_TURN_ACCEPT_ERR_DEG 4.0f
#define NAV_ROW_TURN_FORCE_MS  2600u
#define NAV_ROW_TURN_FORCE_ERR_DEG 7.0f

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

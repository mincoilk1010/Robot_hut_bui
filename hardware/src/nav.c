#include "nav.h"
#include "control.h"
#include "vl53_scan.h"
#include "hc_sr04.h"
#include "avoid.h"
#include "encoder.h"
#include <math.h>

typedef enum {
    SCAN_OBJECT = 0,
    SCAN_WALL_END,
    SCAN_WALL_SIDE
} ScanClass_t;

typedef enum {
    SCAN_OBSTACLE = 0,
    SCAN_RECOVER,
    SCAN_SEARCH
} ScanReason_t;

Nav_t nav = {
    .st = N_BOOT,
    .dir = 1,
    .row = 0u,
    .lane_yaw = 0.0f,
    .x0 = 0.0f,
    .y0 = 0.0f,
    .t0 = 0u,
    .done = 0u
};

static float nav_v_cmd = 0.0f;
static float mark_x = 0.0f;
static float mark_y = 0.0f;

static u8 front_obs_count = 0u;
static u8 front_contact_count = 0u;
static u16 front_hold_mm = 9999u;
static u32 last_front_seq = 0u;

static float pending_turn_yaw = 0.0f;
static NavSt_t pending_after_turn = N_FWD;
static u8 turn_retry_count = 0u;
static u8 pending_turn_is_row = 0u;

static i8 row_shift_side = 1;
static float row_next_yaw = 0.0f;
static float row_shift_target_m = NAV_ROW_M;

static float avoid_resume_yaw = 0.0f;
static float avoid_resume_x0 = 0.0f;
static float avoid_resume_y0 = 0.0f;
static i8 avoid_side = 1;
static float avoid_angle = NAV_AVOID_ANGLE_DEG;
static float rejoin_start_error = 0.0f;

static ScanReason_t scan_reason = SCAN_OBSTACLE;
static u8 no_gap_backup_done = 0u;
static i8 search_dir = 0;
static u8 search_step_count = 0u;

static u32 stuck_watch_ms = 0u;
static float stuck_watch_x = 0.0f;
static float stuck_watch_y = 0.0f;
static u32 hidden_row_watch_ms = 0u;
static float hidden_row_watch_x = 0.0f;
static float hidden_row_watch_y = 0.0f;

static u32 hc_seen_ms[2] = {0u, 0u};
static u8 hc_bad_count[2] = {0u, 0u};
static u8 cliff_latched_mask = 0u;

static float norm_deg(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

static float cross_error_for(float lane_yaw, float x0, float y0)
{
    float psi = DEG2RAD(lane_yaw);
    float dx = pose.x - x0;
    float dy = pose.y - y0;
    return -sinf(psi) * dx + cosf(psi) * dy;
}

static float distance_from_mark(void)
{
    float dx = pose.x - mark_x;
    float dy = pose.y - mark_y;
    return sqrtf(dx * dx + dy * dy);
}

static void mark_position(void)
{
    mark_x = pose.x;
    mark_y = pose.y;
}

static void stop_motion(void)
{
    g_sp_v = 0.0f;
    g_sp_w = 0.0f;
    HeadingHold_SetTrim(0.0f);
}

static void reset_front_watch(void)
{
    front_obs_count = 0u;
    front_contact_count = 0u;
    front_hold_mm = 9999u;
    last_front_seq = scanner_front_seq();
}

static void reset_stuck_watch(void)
{
    stuck_watch_ms = HAL_GetTick();
    stuck_watch_x = pose.x;
    stuck_watch_y = pose.y;
}

static void reset_hidden_row_watch(void)
{
    hidden_row_watch_ms = HAL_GetTick();
    hidden_row_watch_x = pose.x;
    hidden_row_watch_y = pose.y;
}

static i16 drive_pwm_abs_max(void)
{
    i16 abs_l = (p_l < 0) ? (i16)(-p_l) : p_l;
    i16 abs_r = (p_r < 0) ? (i16)(-p_r) : p_r;
    return (abs_l > abs_r) ? abs_l : abs_r;
}

static u8 pwm_force_active(i16 min_pwm)
{
    return (drive_pwm_abs_max() >= min_pwm) ? 1u : 0u;
}

static u8 stuck_pwm_force_active(void)
{
    return pwm_force_active(NAV_STUCK_PWM_MIN);
}

static void reset_search_sweep(void)
{
    search_dir = 0;
    search_step_count = 0u;
}

static void mission_set_line(float yaw_deg, float x0, float y0)
{
    nav.lane_yaw = norm_deg(yaw_deg);
    nav.x0 = x0;
    nav.y0 = y0;
    HeadingHold_SetTarget(nav.lane_yaw);
}

static void enter_state(NavSt_t state)
{
    nav.st = state;
    nav.t0 = HAL_GetTick();
    nav_v_cmd = 0.0f;
    nav.done = (state == N_DONE) ? 1u : 0u;
    HeadingHold_Enable(state != N_DONE);
    reset_stuck_watch();
    reset_hidden_row_watch();

    if (state == N_ROW_CROSS || state == N_AVOID_OFFSET ||
        state == N_AVOID_PASS || state == N_AVOID_REJOIN ||
        state == N_STUCK_BACK || state == N_CLIFF_BACK ||
        state == N_CLIFF_ESCAPE) {
        mark_position();
    }

    if (state == N_AVOID_REJOIN) {
        rejoin_start_error =
            cross_error_for(avoid_resume_yaw, avoid_resume_x0, avoid_resume_y0);
    }

    if (state == N_FWD) {
        reset_front_watch();
        HeadingHold_SetTarget(nav.lane_yaw);
        no_gap_backup_done = 0u;
        reset_search_sweep();
    } else if (state == N_AVOID_OFFSET || state == N_AVOID_PASS) {
        front_hold_mm = 9999u;
        last_front_seq = scanner_front_seq();
    }

    if (state == N_BRAKE || state == N_TURN_WAIT ||
        state == N_CLIFF_CONFIRM || state == N_CLIFF_DECIDE ||
        state == N_CLIFF_SCAN || state == N_DONE) {
        stop_motion();
    }

    if (state == N_BRAKE) {
        scanner_request_wide();
    }
}

static float ramp_speed(float target)
{
    float step = NAV_ACCEL_MPS2 * dt_s;
    float dv = target - nav_v_cmd;
    if (dv > step) dv = step;
    if (dv < -step) dv = -step;
    nav_v_cmd += dv;
    return nav_v_cmd;
}

static float lane_progress(void)
{
    float psi = DEG2RAD(nav.lane_yaw);
    float dx = pose.x - nav.x0;
    float dy = pose.y - nav.y0;
    return cosf(psi) * dx + sinf(psi) * dy;
}

static float lane_cross_error(void)
{
    return cross_error_for(nav.lane_yaw, nav.x0, nav.y0);
}

static void drive_lane(float speed)
{
    float trim = -NAV_XTRACK_K * lane_cross_error();
    trim = limit(trim, -NAV_XTRACK_W_MAX, NAV_XTRACK_W_MAX);
    HeadingHold_SetTrim(trim);
    g_sp_v = ramp_speed(speed);
}

static void drive_heading(float speed)
{
    HeadingHold_SetTrim(0.0f);
    g_sp_v = ramp_speed(speed);
}

static float speed_for_front(u16 front_mm)
{
    if (front_mm >= NAV_OBS_SLOW_MM)
        return NAV_SPEED;
    if (front_mm <= NAV_OBS_TURN_MM)
        return NAV_SPEED * 0.25f;

    float span = (float)(NAV_OBS_SLOW_MM - NAV_OBS_TURN_MM);
    float k = (float)(front_mm - NAV_OBS_TURN_MM) / span;
    return limit(NAV_SPEED * (0.35f + 0.65f * k), 0.04f, NAV_SPEED);
}

static u8 take_new_front(u16 *front_mm)
{
    u32 seq = scanner_front_seq();
    if (seq == last_front_seq)
        return 0u;
    last_front_seq = seq;
    *front_mm = scanner_front();
    return 1u;
}

static void update_cliff_filter(void)
{
    u32 now = HAL_GetTick();
    for (u8 i = 0u; i < 2u; i++) {
        if (hc[i].sample_ms == 0u || hc[i].sample_ms == hc_seen_ms[i])
            continue;

        hc_seen_ms[i] = hc[i].sample_ms;
        if ((u32)(now - hc[i].sample_ms) > HC_SAMPLE_MAX_AGE_MS)
            continue;

        if (hc[i].d > CLIFF_MM) {
            if (hc_bad_count[i] < CLIFF_CONFIRM_SAMPLES)
                hc_bad_count[i]++;
        } else {
            hc_bad_count[i] = 0u;
        }
    }
}

static u8 cliff_mask(void)
{
    if (!NAV_CLIFF_ENABLE)
        return 0u;

    u32 now = HAL_GetTick();
    u8 mask = 0u;

    if (hc[HC_LEFT_INDEX].sample_ms != 0u &&
        (u32)(now - hc[HC_LEFT_INDEX].sample_ms) <= HC_SAMPLE_MAX_AGE_MS &&
        hc_bad_count[HC_LEFT_INDEX] >= CLIFF_CONFIRM_SAMPLES)
        mask |= 1u;

    if (hc[HC_RIGHT_INDEX].sample_ms != 0u &&
        (u32)(now - hc[HC_RIGHT_INDEX].sample_ms) <= HC_SAMPLE_MAX_AGE_MS &&
        hc_bad_count[HC_RIGHT_INDEX] >= CLIFF_CONFIRM_SAMPLES)
        mask |= 2u;

    return mask;
}

static u8 is_moving_state(NavSt_t state)
{
    return state == N_FWD || state == N_ROW_CROSS ||
           state == N_AVOID_OFFSET || state == N_AVOID_PASS ||
           state == N_AVOID_REJOIN;
}

static u8 stuck_detected(u32 now, float cmd_speed)
{
    if (!NAV_STUCK_ENABLE)
        return 0u;

    /* Only back up when the drivetrain is really pushing hard. Seeing a wall
     * or a close obstacle is not "stuck"; PWM around NAV_STUCK_PWM_MIN with
     * no wheel/pose progress is the stuck condition. */
    if (!stuck_pwm_force_active()) {
        reset_stuck_watch();
        return 0u;
    }

    if (fabsf(cmd_speed) < NAV_STUCK_CMD_MIN_MPS) {
        reset_stuck_watch();
        return 0u;
    }

    float wheel_speed = (fabsf(ec_l.vel) + fabsf(ec_r.vel)) * 0.5f;
    float dx = pose.x - stuck_watch_x;
    float dy = pose.y - stuck_watch_y;
    float moved = sqrtf(dx * dx + dy * dy);

    if (wheel_speed > NAV_STUCK_VEL_MAX_MPS ||
        moved >= NAV_STUCK_PROGRESS_MIN_M) {
        reset_stuck_watch();
        return 0u;
    }

    return ((u32)(now - stuck_watch_ms) >= NAV_STUCK_TIME_MS) ? 1u : 0u;
}

static u8 hidden_obstacle_row_detected(u32 now, float cmd_speed)
{
    if (!NAV_STUCK_ENABLE)
        return 0u;

    if (fabsf(cmd_speed) < NAV_STUCK_CMD_MIN_MPS ||
        !pwm_force_active(NAV_HIDDEN_ROW_PWM_MIN) ||
        front_hold_mm <= NAV_OBS_TURN_MM) {
        reset_hidden_row_watch();
        return 0u;
    }

    float wheel_speed = (fabsf(ec_l.vel) + fabsf(ec_r.vel)) * 0.5f;
    float dx = pose.x - hidden_row_watch_x;
    float dy = pose.y - hidden_row_watch_y;
    float moved = sqrtf(dx * dx + dy * dy);

    if (wheel_speed > NAV_STUCK_VEL_MAX_MPS ||
        moved >= NAV_STUCK_PROGRESS_MIN_M) {
        reset_hidden_row_watch();
        return 0u;
    }

    return ((u32)(now - hidden_row_watch_ms) >= NAV_STUCK_TIME_MS) ? 1u : 0u;
}

static void schedule_turn_to_ex(float target_yaw, NavSt_t after_turn,
                                u8 is_row_turn)
{
    pending_turn_yaw = norm_deg(target_yaw);
    pending_after_turn = after_turn;
    pending_turn_is_row = is_row_turn ? 1u : 0u;
    turn_retry_count = 0u;
    stop_motion();
    enter_state(N_TURN_WAIT);
}

static void schedule_turn_to(float target_yaw, NavSt_t after_turn)
{
    schedule_turn_to_ex(target_yaw, after_turn, 0u);
}

static void schedule_row_turn_to(float target_yaw, NavSt_t after_turn)
{
    schedule_turn_to_ex(target_yaw, after_turn, 1u);
}

static u8 turn_timeout_can_continue(float remaining_deg)
{
    if (remaining_deg <= NAV_TURN_ACCEPT_ERR_DEG)
        return 1u;

    if (pending_turn_is_row &&
        remaining_deg <= NAV_ROW_TURN_ACCEPT_ERR_DEG)
        return 1u;

    if ((pending_after_turn == N_ROW_CROSS ||
         pending_after_turn == N_AVOID_PASS ||
         pending_after_turn == N_AVOID_REJOIN ||
         pending_after_turn == N_FWD ||
         pending_after_turn == N_BRAKE) &&
        remaining_deg <= NAV_TURN_COARSE_ACCEPT_ERR_DEG)
        return 1u;

    return 0u;
}

static u8 scan_blocked_near(void)
{
    u8 near_count = 0u;
    u8 front_count = 0u;

    for (u8 deg = SC_W_MIN; deg <= SC_W_MAX; deg += SC_STEP) {
        u16 mm = sc.data[deg / SC_STEP];
        if (mm == 0u || mm == 9999u)
            continue;

        if (mm <= NAV_BLOCKED_NEAR_MM) {
            near_count++;
            if (deg >= 70u && deg <= 110u)
                front_count++;
        }
    }

    return (near_count >= NAV_BLOCKED_NEAR_POINTS &&
            front_count >= NAV_BLOCKED_FRONT_POINTS) ? 1u : 0u;
}

static u8 find_open_gap(i8 *side_out, float *angle_out)
{
    Gap_t gap;
    if (!Avoid_FindBestGap(&gap))
        return 0u;

    i8 side;
    if (fabsf(gap.redirect_deg) < 1.0f)
        side = nav.dir;
    else
        side = (gap.redirect_deg > 0.0f) ? 1 : -1;

    if (side_out) *side_out = side;
    if (angle_out) *angle_out = NAV_AVOID_ANGLE_DEG;
    return 1u;
}

static i8 choose_open_side(void)
{
    i8 side;
    if (find_open_gap(&side, 0))
        return side;

    long sum_left = 0;
    long sum_right = 0;
    int count_left = 0;
    int count_right = 0;

    for (u8 deg = SC_W_MIN; deg <= SC_W_MAX; deg += SC_STEP) {
        u16 mm = sc.data[deg / SC_STEP];
        if (mm == 9999u) mm = 1200u;
        if (deg > 100u) {
            sum_left += mm;
            count_left++;
        } else if (deg < 80u) {
            sum_right += mm;
            count_right++;
        }
    }

    if (count_left == 0 && count_right == 0)
        return nav.dir;

    return ((count_left ? sum_left / count_left : 0) >=
            (count_right ? sum_right / count_right : 0)) ? 1 : -1;
}

static ScanClass_t classify_scan(void)
{
    int best_start = -1;
    int best_end = -1;
    int best_score = -1;
    int best_touches_front = 0;
    int i = 0;

    while (i < 37) {
        u16 mm = sc.data[i];
        if (mm < 60u || mm > 1200u) {
            i++;
            continue;
        }

        int start = i;
        int touches_front = (i >= 15 && i <= 21);
        u16 previous = mm;
        i++;

        while (i < 37) {
            mm = sc.data[i];
            if (mm < 60u || mm > 1200u)
                break;
            if (ABS_F((float)mm - (float)previous) > NAV_SCAN_SEGMENT_JUMP_MM)
                break;
            if (i >= 15 && i <= 21) touches_front = 1;
            previous = mm;
            i++;
        }

        int end = i - 1;
        int count = end - start + 1;
        int score = count + (touches_front ? 100 : 0);
        if (score > best_score) {
            best_score = score;
            best_start = start;
            best_end = end;
            best_touches_front = touches_front;
        }
    }

    if (best_start < 0 || best_end - best_start + 1 < 3)
        return SCAN_OBJECT;

    int count = best_end - best_start + 1;
    float mean_x = 0.0f;
    float mean_y = 0.0f;
    for (i = best_start; i <= best_end; i++) {
        float range = (float)sc.data[i] * 0.001f;
        float alpha = DEG2RAD((float)(i * SC_STEP) - 90.0f);
        mean_x += range * cosf(alpha);
        mean_y += range * sinf(alpha);
    }
    mean_x /= (float)count;
    mean_y /= (float)count;

    float cxx = 0.0f;
    float cxy = 0.0f;
    float cyy = 0.0f;
    for (i = best_start; i <= best_end; i++) {
        float range = (float)sc.data[i] * 0.001f;
        float alpha = DEG2RAD((float)(i * SC_STEP) - 90.0f);
        float dx = range * cosf(alpha) - mean_x;
        float dy = range * sinf(alpha) - mean_y;
        cxx += dx * dx;
        cxy += dx * dy;
        cyy += dy * dy;
    }
    cxx /= (float)count;
    cxy /= (float)count;
    cyy /= (float)count;

    float axis = 0.5f * atan2f(2.0f * cxy, cxx - cyy);
    float vx = cosf(axis);
    float vy = sinf(axis);
    float trace = cxx + cyy;
    float disc = sqrtf((cxx - cyy) * (cxx - cyy) + 4.0f * cxy * cxy);
    float lambda_min = 0.5f * (trace - disc);
    if (lambda_min < 0.0f) lambda_min = 0.0f;

    float proj_min = 9999.0f;
    float proj_max = -9999.0f;
    for (i = best_start; i <= best_end; i++) {
        float range = (float)sc.data[i] * 0.001f;
        float alpha = DEG2RAD((float)(i * SC_STEP) - 90.0f);
        float x = range * cosf(alpha) - mean_x;
        float y = range * sinf(alpha) - mean_y;
        float p = x * vx + y * vy;
        if (p < proj_min) proj_min = p;
        if (p > proj_max) proj_max = p;
    }

    float length = proj_max - proj_min;
    float line_rms = sqrtf(lambda_min);
    float seg_start_deg = (float)(best_start * SC_STEP);
    float seg_end_deg = (float)(best_end * SC_STEP);
    u8 spans_wall_width =
        (seg_start_deg <= NAV_WALL_SPAN_LEFT_DEG &&
         seg_end_deg >= NAV_WALL_SPAN_RIGHT_DEG);

    if (count >= NAV_WALL_MIN_POINTS &&
        length >= NAV_WALL_MIN_LENGTH_M &&
        line_rms <= NAV_WALL_LINE_RMS_MAX_M &&
        (spans_wall_width || best_touches_front)) {
        if (spans_wall_width ||
            scan_blocked_near() ||
            fabsf(vx) < NAV_WALL_END_AXIS_MAX) {
            return SCAN_WALL_END;
        }
        return SCAN_WALL_SIDE;
    }

    return SCAN_OBJECT;
}

static void begin_scan(ScanReason_t reason)
{
    scan_reason = reason;
    stop_motion();
    enter_state(N_BRAKE);
}

static void begin_recover(void)
{
    scan_reason = SCAN_RECOVER;
    HeadingHold_SetTarget(yaw);
    stop_motion();
    enter_state(N_STUCK_BACK);
}

static void begin_row_change_ex(i8 side, float shift_m)
{
    no_gap_backup_done = 0u;
    reset_search_sweep();

    if ((u8)(nav.row + 1u) >= NAV_MAX_ROWS) {
        enter_state(N_DONE);
        return;
    }

    row_shift_side = (side >= 0) ? 1 : -1;
    row_next_yaw = norm_deg(nav.lane_yaw + 180.0f);
    row_shift_target_m = shift_m;
    schedule_row_turn_to(nav.lane_yaw + 90.0f * (float)row_shift_side,
                         N_ROW_CROSS);
}

static void begin_row_change(i8 side)
{
    begin_row_change_ex(side, NAV_ROW_M);
}

static void begin_wall_row_change(i8 side)
{
    begin_row_change_ex(side, NAV_WALL_ROW_M);
}

static void begin_search_turn(void)
{
    if (search_dir == 0) {
        search_dir = choose_open_side();
        if (search_dir == 0)
            search_dir = (nav.dir >= 0) ? 1 : -1;
    }

    if (search_step_count >= NAV_SEARCH_MAX_STEPS) {
        search_dir = (i8)(-search_dir);
        search_step_count = 0u;
    }

    search_step_count++;
    scan_reason = SCAN_SEARCH;

    float search_yaw =
        norm_deg(yaw + (float)search_dir * NAV_SEARCH_STEP_DEG);
    schedule_turn_to(search_yaw, N_BRAKE);
}

static void begin_avoid_with(i8 side, float angle)
{
    avoid_resume_yaw = nav.lane_yaw;
    avoid_resume_x0 = nav.x0;
    avoid_resume_y0 = nav.y0;
    avoid_side = (side >= 0) ? 1 : -1;
    avoid_angle = angle;

    no_gap_backup_done = 0u;
    reset_search_sweep();

    schedule_turn_to(avoid_resume_yaw +
                     (float)avoid_side * avoid_angle,
                     N_AVOID_OFFSET);
}

static void handle_scan_result(void)
{
    ScanClass_t kind = scanner_wide_ready() ? classify_scan() : SCAN_OBJECT;
    i8 side = nav.dir;
    float angle = NAV_AVOID_ANGLE_DEG;
    u8 has_gap = scanner_wide_ready() ? find_open_gap(&side, &angle) : 0u;
    u8 wall_blocks_front =
        (kind == SCAN_WALL_END ||
         (kind == SCAN_WALL_SIDE && scan_blocked_near())) ? 1u : 0u;

    if (wall_blocks_front) {
        i8 row_side = has_gap ? side : choose_open_side();
        scanner_wide_consume();
        begin_wall_row_change(row_side);
        return;
    }

    if (has_gap) {
        scanner_wide_consume();
        begin_avoid_with(side, angle);
        return;
    }

    if (scanner_wide_ready() && scan_blocked_near() && !no_gap_backup_done) {
        no_gap_backup_done = 1u;
        scanner_wide_consume();
        begin_search_turn();
        return;
    }

    scanner_wide_consume();
    begin_search_turn();
}

void nav_init(void)
{
    nav.st = N_BOOT;
    nav.dir = 1;
    nav.row = 0u;
    nav.done = 0u;
    nav.t0 = HAL_GetTick();

    nav_v_cmd = 0.0f;
    reset_front_watch();
    reset_search_sweep();
    no_gap_backup_done = 0u;
    hc_seen_ms[0] = hc_seen_ms[1] = 0u;
    hc_bad_count[0] = hc_bad_count[1] = 0u;

    mission_set_line(yaw, pose.x, pose.y);
    HeadingHold_SetTrim(0.0f);
    HeadingHold_Enable(1u);
}

void nav_task(void)
{
    u32 now = HAL_GetTick();
    update_cliff_filter();

    u8 mask = cliff_mask();
    if (mask != 0u && is_moving_state(nav.st)) {
        cliff_latched_mask = mask;
        begin_recover();
        return;
    }

    switch (nav.st) {
    case N_BOOT:
        stop_motion();
        if ((u32)(now - nav.t0) >= 800u) {
            mission_set_line(yaw, pose.x, pose.y);
            enter_state(N_FWD);
        }
        break;

    case N_FWD: {
        u16 front;
        if (take_new_front(&front)) {
            front_hold_mm = front;
            if (front <= NAV_OBS_CONTACT_MM) {
                if (front_contact_count < 3u) front_contact_count++;
                if (front_obs_count < 3u) front_obs_count++;
            } else if (front <= NAV_OBS_TURN_MM) {
                front_contact_count = 0u;
                if (front_obs_count < 3u) front_obs_count++;
            } else {
                front_contact_count = 0u;
                front_obs_count = 0u;
            }
        }

        if (lane_progress() >= NAV_ROW_LENGTH_M) {
            begin_row_change(nav.dir);
        } else if (front_contact_count >= 2u) {
            reset_front_watch();
            begin_scan(SCAN_OBSTACLE);
        } else if (front_obs_count >= 2u) {
            reset_front_watch();
            begin_scan(SCAN_OBSTACLE);
        } else {
            drive_lane(speed_for_front(front_hold_mm));
            if (hidden_obstacle_row_detected(now, g_sp_v))
                begin_row_change(nav.dir);
            else if (stuck_detected(now, g_sp_v))
                begin_recover();
        }
        break;
    }

    case N_BRAKE:
        stop_motion();
        if ((u32)(now - nav.t0) < 200u)
            break;

        if (scanner_wide_ready() ||
            (u32)(now - nav.t0) >= NAV_SCAN_TIMEOUT_MS) {
            handle_scan_result();
        }
        break;

    case N_TURN_WAIT:
        stop_motion();
        if ((u32)(now - nav.t0) >= NAV_STOP_SETTLE_MS) {
            turn_start_to(pending_turn_yaw);
            enter_state(N_TURN_ACTIVE);
        }
        break;

    case N_TURN_ACTIVE:
        if (turn_done()) {
            if (turn.timeout) {
                float remaining = fabsf(angle_diff(pending_turn_yaw, yaw));
                if (turn_timeout_can_continue(remaining)) {
                    HeadingHold_SetTarget(pending_turn_yaw);
                    enter_state(pending_after_turn);
                } else if (turn_retry_count <
                           (pending_turn_is_row ? NAV_ROW_TURN_RETRY_MAX
                                                : NAV_TURN_RETRY_MAX)) {
                    turn_retry_count++;
                    HeadingHold_SetTarget(yaw);
                    enter_state(N_TURN_WAIT);
                } else if (stuck_pwm_force_active()) {
                    begin_recover();
                } else {
                    begin_scan(SCAN_SEARCH);
                }
                break;
            }

            HeadingHold_SetTarget(pending_turn_yaw);
            turn_retry_count = 0u;
            enter_state(pending_after_turn);
        }
        break;

    case N_ROW_CROSS:
        drive_heading(NAV_SPEED * 0.8f);
        if ((u32)(now - nav.t0) >= NAV_ROW_CROSS_STUCK_GRACE_MS &&
            stuck_detected(now, g_sp_v)) {
            begin_recover();
            break;
        }
        if (distance_from_mark() >= row_shift_target_m) {
            nav.row++;
            nav.dir = (i8)(-row_shift_side);
            mission_set_line(row_next_yaw, pose.x, pose.y);
            schedule_row_turn_to(nav.lane_yaw, N_FWD);
        }
        break;

    case N_AVOID_OFFSET:
        drive_heading(NAV_SPEED * 0.7f);
        if (stuck_detected(now, g_sp_v)) {
            begin_recover();
            break;
        }
        {
            u16 front;
            if (take_new_front(&front) && front <= NAV_OBS_CONTACT_MM) {
                begin_scan(SCAN_OBSTACLE);
            } else if (distance_from_mark() >= NAV_AVOID_OFFSET_M) {
                schedule_turn_to(avoid_resume_yaw, N_AVOID_PASS);
            }
        }
        break;

    case N_AVOID_PASS:
        drive_heading(NAV_SPEED * 0.75f);
        if (stuck_detected(now, g_sp_v)) {
            begin_recover();
            break;
        }
        {
            u16 front;
            if (take_new_front(&front))
                front_hold_mm = front;

            if (front_hold_mm <= NAV_OBS_CONTACT_MM) {
                begin_scan(SCAN_OBSTACLE);
            } else if (distance_from_mark() >= NAV_AVOID_PASS_M &&
                       front_hold_mm > NAV_OBS_TURN_MM) {
                schedule_turn_to(avoid_resume_yaw -
                                 (float)avoid_side * avoid_angle,
                                 N_AVOID_REJOIN);
            }
        }
        break;

    case N_AVOID_REJOIN: {
        drive_heading(NAV_SPEED * 0.45f);
        if (stuck_detected(now, g_sp_v)) {
            begin_recover();
            break;
        }

        float e = cross_error_for(avoid_resume_yaw,
                                  avoid_resume_x0,
                                  avoid_resume_y0);
        float travelled = distance_from_mark();
        u8 near_lane = (fabsf(e) <= NAV_AVOID_REJOIN_LEAD_M);
        u8 crossed_lane =
            (fabsf(rejoin_start_error) > NAV_AVOID_REJOIN_LEAD_M &&
             (rejoin_start_error * e) <= 0.0f);

        if ((travelled > 0.10f && (near_lane || crossed_lane)) ||
            travelled >= NAV_AVOID_REJOIN_M) {
            mission_set_line(avoid_resume_yaw,
                             avoid_resume_x0,
                             avoid_resume_y0);
            schedule_turn_to(nav.lane_yaw, N_FWD);
        }
        break;
    }

    case N_STUCK_BACK:
        drive_heading(-NAV_STUCK_BACK_SPEED);
        if (distance_from_mark() >= NAV_STUCK_BACK_M ||
            (u32)(now - nav.t0) >= NAV_STUCK_BACK_TIMEOUT_MS) {
            begin_scan(SCAN_RECOVER);
        }
        break;

    case N_CLIFF_CONFIRM:
    case N_CLIFF_BACK:
    case N_CLIFF_DECIDE:
    case N_CLIFF_SCAN:
    case N_CLIFF_ESCAPE:
        /* HC-SR04 cliff is disabled in the current hardware setup. If it is
         * enabled later, keep recovery separate from the mission line. */
        begin_recover();
        break;

    case N_DONE:
    default:
        stop_motion();
        nav.done = 1u;
        break;
    }
}

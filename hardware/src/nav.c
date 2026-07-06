#include "nav.h"
#include "control.h"
#include "vl53_scan.h"
#include "avoid.h"
#include "encoder.h"
#include <math.h>

typedef enum {
    SCENE_OBJECT = 0,
    SCENE_WALL,
    SCENE_UNKNOWN
} Scene_t;

typedef struct {
    u8 found;
    float width_m;
    float front_m;
    float center_deg;
    u8 start_deg;
    u8 end_deg;
} ObjectMeasure_t;

typedef struct {
    u8 found;
    int count;
    int front_count;
    int first_deg;
    int last_deg;
    int span_deg;
    u16 min_mm;
    u16 avg_mm;
} FrontBlock_t;

typedef struct {
    u8 valid;
    int valid_count;
    int near_count;
    int front_near_count;
    int first_near_deg;
    int last_near_deg;
    int near_span_deg;
    u16 min_mm;
    u16 max_mm;
    u16 spread_mm;
    u16 avg_mm;
} ScanShape_t;

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
static float mark_l = 0.0f;
static float mark_r = 0.0f;

static float pending_turn_yaw = 0.0f;
static NavSt_t pending_after_turn = N_FWD;
static u8 pending_turn_is_row = 0u;

static i8 row_shift_side = 1;
static float row_next_yaw = 0.0f;
static float row_shift_target_m = NAV_WALL_ROW_M;
static u8 wall_row_active = 0u;
static u8 object_avoid_active = 0u;
static float row_cross_l0 = 0.0f;
static float row_cross_r0 = 0.0f;

static float avoid_resume_yaw = 0.0f;
static float avoid_resume_x0 = 0.0f;
static float avoid_resume_y0 = 0.0f;
static i8 avoid_side = 1;
static float avoid_offset_m = NAV_AVOID_OFFSET_M;
static float avoid_actual_offset_m = NAV_AVOID_OFFSET_M;
static float avoid_pass_m = NAV_AVOID_PASS_M;
static u8 avoid_pass_phase = 0u;
static u8 avoid_side_seen = 0u;
static u8 avoid_side_lost_count = 0u;
static float rejoin_start_error = 0.0f;

static u16 front_hold_mm = 9999u;
static u32 last_front_seq = 0u;

static Scene_t planned_scene = SCENE_UNKNOWN;
static i8 planned_side = 1;
static u8 planned_ready = 0u;
static u8 scan_for_plan = 0u;
static u16 planned_front_mm = 9999u;
static float planned_x = 0.0f;
static float planned_y = 0.0f;
static float planned_run_m = 0.0f;
static float planned_object_width_m = 0.0f;
static float planned_avoid_offset_m = NAV_AVOID_OFFSET_M;
static float planned_avoid_pass_m = NAV_AVOID_PASS_M;

static u32 stuck_watch_ms = 0u;
static float stuck_watch_x = 0.0f;
static float stuck_watch_y = 0.0f;

static float norm_deg(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

static float yaw_delta_for_side(i8 side, float deg)
{
    i8 s = (side >= 0) ? 1 : -1;
    return NAV_YAW_LEFT_SIGN * (float)s * deg;
}

static i8 avoid_side_from_obstacle_offset(float offset_deg)
{
    if (offset_deg > NAV_OBJECT_CENTER_BIAS_DEG) {
        /* Obstacle is on the high-angle side of the scan. */
        return NAV_SCAN_LEFT_IS_HIGH_DEG ? -1 : 1;
    }

    if (offset_deg < -NAV_OBJECT_CENTER_BIAS_DEG) {
        /* Obstacle is on the low-angle side of the scan. */
        return NAV_SCAN_LEFT_IS_HIGH_DEG ? 1 : -1;
    }

    return 0;
}

static float cross_error_for(float lane_yaw, float x0, float y0)
{
    float psi = DEG2RAD(lane_yaw);
    float dx = pose.x - x0;
    float dy = pose.y - y0;
    return -sinf(psi) * dx + cosf(psi) * dy;
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

static void mark_position(void)
{
    mark_x = pose.x;
    mark_y = pose.y;
    mark_l = ec_l.dist;
    mark_r = ec_r.dist;
}

static float distance_from_mark(void)
{
    float dx = pose.x - mark_x;
    float dy = pose.y - mark_y;
    return sqrtf(dx * dx + dy * dy);
}

static float encoder_distance_from_mark(void)
{
    float dl = ABS_F(ec_l.dist - mark_l);
    float dr = ABS_F(ec_r.dist - mark_r);
    return (dl + dr) * 0.5f;
}

static float row_cross_encoder_distance(void)
{
    float dl = ABS_F(ec_l.dist - row_cross_l0);
    float dr = ABS_F(ec_r.dist - row_cross_r0);
    return (dl + dr) * 0.5f;
}

static void stop_motion(void)
{
    g_sp_v = 0.0f;
    g_sp_w = 0.0f;
    nav_v_cmd = 0.0f;
    HeadingHold_SetTrim(0.0f);
}

static void reset_front_watch(void)
{
    front_hold_mm = 9999u;
    last_front_seq = scanner_front_seq();
}

static void reset_stuck_watch(void)
{
    stuck_watch_ms = HAL_GetTick();
    stuck_watch_x = pose.x;
    stuck_watch_y = pose.y;
}

static void clear_obstacle_plan(void)
{
    planned_scene = SCENE_UNKNOWN;
    planned_side = nav.dir;
    planned_ready = 0u;
    scan_for_plan = 0u;
    planned_front_mm = 9999u;
    planned_x = pose.x;
    planned_y = pose.y;
    planned_run_m = 0.0f;
    planned_object_width_m = 0.0f;
    planned_avoid_offset_m = NAV_AVOID_OFFSET_M;
    planned_avoid_pass_m = NAV_AVOID_PASS_M;
}

static void resume_forward_keep_plan(void)
{
    nav.st = N_FWD;
    nav.t0 = HAL_GetTick();
    nav.done = 0u;
    nav_v_cmd = 0.0f;
    reset_stuck_watch();
    HeadingHold_SetTarget(nav.lane_yaw);
    HeadingHold_SetTrim(0.0f);
}

static float planned_distance_done(void)
{
    float dx = pose.x - planned_x;
    float dy = pose.y - planned_y;
    return sqrtf(dx * dx + dy * dy);
}

static u8 object_side_servo_deg(void)
{
    /* avoid_side > 0: robot turns left, object is on robot right side.
     * avoid_side < 0: robot turns right, object is on robot left side.
     */
    return (avoid_side > 0) ? NAV_SIDE_LOOK_RIGHT_DEG
                            : NAV_SIDE_LOOK_LEFT_DEG;
}

static u8 side_sensor_sees_object(void)
{
    u16 mm = scanner_locked_mm();
    return (mm != 9999u && mm <= NAV_SIDE_OBJECT_MM) ? 1u : 0u;
}

static u8 scan_mm_is_near(u16 mm, u16 near_limit_mm)
{
    return (mm != 0u && mm != 9999u && mm <= near_limit_mm) ? 1u : 0u;
}

static void mission_set_line(float yaw_deg, float x0, float y0)
{
    nav.lane_yaw = norm_deg(yaw_deg);
    nav.x0 = x0;
    nav.y0 = y0;
    HeadingHold_SetTarget(nav.lane_yaw);
    HeadingHold_SetTrim(0.0f);
}

static void enter_state(NavSt_t state)
{
    nav.st = state;
    nav.t0 = HAL_GetTick();
    nav.done = (state == N_DONE) ? 1u : 0u;
    nav_v_cmd = 0.0f;
    reset_stuck_watch();

    if (state == N_FWD) {
        wall_row_active = 0u;
        object_avoid_active = 0u;
        scanner_pause(0u);
        scanner_unlock();
        reset_front_watch();
        clear_obstacle_plan();
        if (scanner_mode() == SC_WIDE)
            scanner_wide_consume();
        HeadingHold_SetTarget(nav.lane_yaw);
        HeadingHold_SetTrim(0.0f);
    }

    if (state == N_BRAKE || state == N_TURN_WAIT ||
        state == N_TURN_ACTIVE || state == N_DONE) {
        stop_motion();
    }

    if (state == N_BRAKE) {
        scanner_pause(0u);
        scanner_unlock();
        scanner_request_wide();
    }

    if (state == N_ROW_CROSS || state == N_AVOID_OFFSET ||
        state == N_AVOID_PASS || state == N_AVOID_REJOIN ||
        state == N_STUCK_BACK) {
        mark_position();
    }

    if (state == N_ROW_CROSS) {
        row_cross_l0 = ec_l.dist;
        row_cross_r0 = ec_r.dist;
    }

    if (state == N_AVOID_OFFSET || state == N_AVOID_PASS ||
        state == N_AVOID_REJOIN) {
        reset_front_watch();
        front_hold_mm = 9999u;
    }

    u8 avoid_turn =
        (state == N_TURN_WAIT &&
         (pending_after_turn == N_AVOID_OFFSET ||
          pending_after_turn == N_AVOID_PASS ||
          pending_after_turn == N_AVOID_REJOIN)) ? 1u : 0u;

    if (!wall_row_active && !object_avoid_active &&
        ((state == N_TURN_WAIT && !avoid_turn) || state == N_ROW_CROSS ||
         state == N_AVOID_REJOIN || state == N_STUCK_BACK ||
         state == N_DONE)) {
        scanner_unlock();
    }

    if (state == N_AVOID_OFFSET) {
        object_avoid_active = 1u;
        scanner_pause(0u);
        avoid_side_seen = 0u;
        avoid_side_lost_count = 0u;
        scanner_lock_angle(object_side_servo_deg());
    }

    if (state == N_AVOID_PASS) {
        object_avoid_active = 1u;
        scanner_pause(0u);
        avoid_pass_phase = 0u;
        avoid_side_seen = 0u;
        avoid_side_lost_count = 0u;
        scanner_lock_angle(object_side_servo_deg());
    }

    if (state == N_AVOID_REJOIN) {
        object_avoid_active = 1u;
        scanner_pause(1u);
        rejoin_start_error =
            cross_error_for(avoid_resume_yaw, avoid_resume_x0, avoid_resume_y0);
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

static void drive_heading(float speed)
{
    HeadingHold_SetTrim(0.0f);
    g_sp_v = ramp_speed(speed);
}

static void drive_lane(float speed)
{
    float e = lane_cross_error();
    if (fabsf(e) < NAV_XTRACK_DEADBAND_M)
        e = 0.0f;

    float trim = -NAV_XTRACK_K * e;
    trim = limit(trim, -NAV_XTRACK_W_MAX, NAV_XTRACK_W_MAX);
    HeadingHold_SetTrim(trim);
    g_sp_v = ramp_speed(speed);
}

static float speed_for_front(u16 front_mm)
{
    if (front_mm >= NAV_OBS_LOOKAHEAD_MM)
        return NAV_SPEED;

    if (front_mm > NAV_OBS_SLOW_MM) {
        float span = (float)(NAV_OBS_LOOKAHEAD_MM - NAV_OBS_SLOW_MM);
        float k = (float)(front_mm - NAV_OBS_SLOW_MM) / span;
        return limit(NAV_SPEED * (0.75f + 0.25f * k),
                     NAV_SPEED * 0.70f, NAV_SPEED);
    }

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

static i16 drive_pwm_abs_max(void)
{
    i16 abs_l = (p_l < 0) ? (i16)(-p_l) : p_l;
    i16 abs_r = (p_r < 0) ? (i16)(-p_r) : p_r;
    return (abs_l > abs_r) ? abs_l : abs_r;
}

static u8 stuck_detected(u32 now, float cmd_speed)
{
    if (!NAV_STUCK_ENABLE)
        return 0u;

    if (fabsf(cmd_speed) < NAV_STUCK_CMD_MIN_MPS ||
        drive_pwm_abs_max() < NAV_STUCK_PWM_MIN) {
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

static u8 wide_scan_usable(void)
{
    return scanner_wide_ready();
}

static i8 side_from_gap(const Gap_t *gap)
{
    if (gap->found) {
        if (gap->redirect_deg > 1.0f)
            return 1;
        if (gap->redirect_deg < -1.0f)
            return -1;
    }
    return nav.dir;
}

static i8 choose_open_side(void)
{
    Gap_t gap;
    if (Avoid_FindBestGap(&gap))
        return side_from_gap(&gap);

    long sum_left = 0;
    long sum_right = 0;
    int cnt_left = 0;
    int cnt_right = 0;

    for (u8 deg = SC_W_MIN; deg <= SC_W_MAX; deg += SC_STEP) {
        u16 mm = scanner_get(deg);
        if (mm == 0u || mm == 9999u)
            continue;

        if (deg >= 105u) {
            sum_left += mm;
            cnt_left++;
        } else if (deg <= 75u) {
            sum_right += mm;
            cnt_right++;
        }
    }

    if (cnt_left == 0 && cnt_right == 0)
        return nav.dir;
    if (cnt_left == 0)
        return -1;
    if (cnt_right == 0)
        return 1;

    return ((sum_left / cnt_left) >= (sum_right / cnt_right)) ? 1 : -1;
}

static int side_near_count(u8 deg_min, u8 deg_max)
{
    int near_cnt = 0;

    for (u8 deg = deg_min; deg <= deg_max; deg += SC_STEP) {
        u16 mm = scanner_get(deg);
        if (mm == 0u || mm == 9999u)
            continue;

        if (mm <= NAV_OBS_LOOKAHEAD_MM)
            near_cnt++;
    }

    return near_cnt;
}

static float scan_obstacle_center_offset_deg(u8 *valid_out)
{
    float weighted_sum = 0.0f;
    float weight_total = 0.0f;
    const u16 near_limit =
        (u16)(NAV_OBS_LOOKAHEAD_MM + NAV_WALL_NEAR_EXTRA_MM);

    for (u8 deg = NAV_SHAPE_SCAN_MIN_DEG;
         deg <= NAV_SHAPE_SCAN_MAX_DEG;
         deg += SC_STEP) {
        u16 mm = scanner_get(deg);
        if (mm == 0u || mm == 9999u || mm > near_limit)
            continue;

        /* Closer points carry more weight.  The result is negative when the
         * obstacle mass is on the right side of the robot, positive when it
         * is on the left side.
         */
        float weight = (float)(near_limit - mm + 1u);
        weighted_sum += ((float)deg - 90.0f) * weight;
        weight_total += weight;
    }

    if (weight_total <= 0.0f) {
        if (valid_out) *valid_out = 0u;
        return 0.0f;
    }

    if (valid_out) *valid_out = 1u;
    return weighted_sum / weight_total;
}

static i8 side_from_blocked_side(void)
{
    /* deg > 90: left side of the scan, deg < 90: right side of the scan.
     * If the obstacle/blocked points are mostly on one side, avoid to the
     * opposite side before considering softer gap scores.
     */
    u8 center_valid = 0u;
    float center_offset = scan_obstacle_center_offset_deg(&center_valid);
    if (center_valid) {
        i8 side = avoid_side_from_obstacle_offset(center_offset);
        if (side != 0)
            return side;
    }

    int left_near = side_near_count(95u, 165u);
    int right_near = side_near_count(15u, 85u);

    if (left_near >= right_near + 1)
        return -1;  /* blocked/object on left  -> avoid right */

    if (right_near >= left_near + 1)
        return 1;   /* blocked/object on right -> avoid left */

    return 0;
}

static float escape_side_score(u8 deg_min, u8 deg_max)
{
    long sum = 0;
    int cnt = 0;
    int clear_cnt = 0;
    int near_cnt = 0;
    int unknown_cnt = 0;
    u16 min_mm = 9999u;

    for (u8 deg = deg_min; deg <= deg_max; deg += SC_STEP) {
        u16 mm = scanner_get(deg);
        if (mm == 0u)
            continue;

        if (mm == 9999u) {
            mm = GAP_UNKNOWN_MM;
            unknown_cnt++;
        } else if (mm < min_mm) {
            min_mm = mm;
        }

        if (mm >= GAP_CLEAR_MM)
            clear_cnt++;
        if (mm <= (NAV_OBS_DECIDE_MM + 120u))
            near_cnt++;

        sum += mm;
        cnt++;
    }

    if (cnt == 0)
        return -10000.0f;

    if (min_mm == 9999u)
        min_mm = GAP_UNKNOWN_MM;

    float avg = (float)sum / (float)cnt;

    /*
     * Score cao = thoang hon.
     * - avg/min lon: ben do rong va xa vat.
     * - clear_cnt lon: nhieu tia du cho xe di.
     * - near_cnt lon: vat dang lech/chan ben do, phai tru nang.
     * - unknown 9999 co the la thoang xa, nhung tru nhe de tranh "ao".
     */
    return avg + 0.25f * (float)min_mm
               + 35.0f * (float)clear_cnt
               - 180.0f * (float)near_cnt
               - 25.0f * (float)unknown_cnt;
}

static i8 choose_object_escape_side(const Gap_t *gap)
{
    i8 gap_side = side_from_gap(gap);

    /* Theo quy uoc scan hien tai:
     * deg lon hon 90 la ben trai, deg nho hon 90 la ben phai.
     *
     * Quyet dinh uu tien:
     * 1) Vat lech ben nao thi ne sang ben nguoc lai.
     * 2) Neu vat gan giua, ben nao that su thoang hon thi chon ben do.
     * 3) Cuoi cung moi dung gap/nav.dir.
     */
    i8 blocked_side = side_from_blocked_side();
    if (blocked_side != 0)
        return blocked_side;

    float left_score = escape_side_score(100u, 165u);
    float right_score = escape_side_score(15u, 80u);
    float diff = left_score - right_score;

    if (diff > 60.0f)
        return 1;
    if (diff < -60.0f)
        return -1;

    return gap_side;
}

static ObjectMeasure_t estimate_object_from_scan(void)
{
    ObjectMeasure_t out = {0};
    int best_start = -1;
    int best_end = -1;
    float best_score = -100000.0f;

    const u16 max_obj_mm = NAV_OBS_LOOKAHEAD_MM + 300u;
    int i = (int)(SC_W_MIN / SC_STEP);
    int n = (int)((SC_W_MAX - SC_W_MIN) / SC_STEP + 1u);

    while (i < n) {
        u16 mm = sc.data[i];
        if (mm == 0u || mm == 9999u || mm > max_obj_mm) {
            i++;
            continue;
        }

        int start = i;
        int end = i;
        int cnt = 1;
        int front_touch = 0;
        long sum = mm;
        u16 prev = mm;
        u16 min_mm = mm;

        u8 deg = (u8)(SC_W_MIN + i * SC_STEP);
        if (deg >= 70u && deg <= 110u)
            front_touch = 1;

        i++;
        while (i < n) {
            mm = sc.data[i];
            if (mm == 0u || mm == 9999u || mm > max_obj_mm)
                break;
            if (ABS_F((float)mm - (float)prev) > NAV_SCAN_SEGMENT_JUMP_MM)
                break;

            deg = (u8)(SC_W_MIN + i * SC_STEP);
            if (deg >= 70u && deg <= 110u)
                front_touch = 1;

            end = i;
            cnt++;
            sum += mm;
            if (mm < min_mm)
                min_mm = mm;
            prev = mm;
            i++;
        }

        if (cnt >= 2) {
            float center_deg =
                (float)(SC_W_MIN + (start + end) * SC_STEP / 2);
            float avg_mm = (float)sum / (float)cnt;
            float score = (float)cnt * 35.0f
                        - avg_mm * 0.15f
                        - ABS_F(center_deg - 90.0f) * 4.0f
                        + (front_touch ? 700.0f : 0.0f)
                        - (float)min_mm * 0.05f;

            if (score > best_score) {
                best_score = score;
                best_start = start;
                best_end = end;
            }
        }
    }

    if (best_start < 0 || best_end < best_start)
        return out;

    float y_min = 9999.0f;
    float y_max = -9999.0f;
    float front_min = 9999.0f;
    float sum_d = 0.0f;
    int cnt = 0;

    for (i = best_start; i <= best_end; i++) {
        u16 mm = sc.data[i];
        if (mm == 0u || mm == 9999u || mm > max_obj_mm)
            continue;

        u8 deg = (u8)(SC_W_MIN + i * SC_STEP);
        float d_m = (float)mm * 0.001f;
        float a = DEG2RAD((float)deg - 90.0f);
        float x = d_m * cosf(a);
        float y = d_m * sinf(a);

        if (y < y_min) y_min = y;
        if (y > y_max) y_max = y;
        if (x < front_min) front_min = x;
        sum_d += d_m;
        cnt++;
    }

    if (cnt < 2)
        return out;

    float avg_d = sum_d / (float)cnt;
    float edge_pad = avg_d * sinf(DEG2RAD((float)SC_STEP)) * 1.5f;
    float width = (y_max - y_min) + 2.0f * edge_pad;
    width = limit(width, NAV_OBJECT_MIN_WIDTH_M, NAV_OBJECT_MAX_WIDTH_M);

    out.found = 1u;
    out.width_m = width;
    out.front_m = front_min;
    out.center_deg =
        (float)SC_W_MIN +
        ((float)(best_start + best_end) * (float)SC_STEP * 0.5f);
    out.start_deg = (u8)(SC_W_MIN + best_start * SC_STEP);
    out.end_deg = (u8)(SC_W_MIN + best_end * SC_STEP);
    return out;
}

static i8 side_from_object_position(const ObjectMeasure_t *obj)
{
    if (!obj->found)
        return 0;

    return avoid_side_from_obstacle_offset(obj->center_deg - 90.0f);
}

static void set_planned_avoid_from_width(float object_width_m)
{
    float half_robot = ROBOT_WIDTH_M * 0.5f;
    float half_obj = object_width_m * 0.5f;

    planned_object_width_m = object_width_m;
    planned_avoid_offset_m =
        limit(half_obj + half_robot + NAV_AVOID_SIDE_SAFE_M,
              NAV_AVOID_OFFSET_MIN_M,
              NAV_AVOID_OFFSET_MAX_M);

    /* With a single VL53 at the front, depth is not directly visible.
     * For box-like objects use front width as a conservative depth estimate,
     * then add robot length and safety distance.
     */
    planned_avoid_pass_m =
        limit(object_width_m + NAV_ROBOT_LENGTH_M + NAV_AVOID_PASS_SAFE_M,
              NAV_AVOID_PASS_MIN_M,
              NAV_AVOID_PASS_MAX_M);
}

static FrontBlock_t front_near_block(u16 near_limit_mm)
{
    FrontBlock_t best = {0};
    float best_score = -100000.0f;
    u8 deg = 40u;

    while (deg <= 140u) {
        u16 mm = scanner_get(deg);
        if (!scan_mm_is_near(mm, near_limit_mm)) {
            deg += SC_STEP;
            continue;
        }

        int start_deg = deg;
        int last_deg = deg;
        int count = 0;
        int front_count = 0;
        long sum = 0;
        u16 min_mm = mm;
        u16 prev_mm = mm;

        while (deg <= 140u) {
            mm = scanner_get(deg);
            if (!scan_mm_is_near(mm, near_limit_mm))
                break;
            if (count > 0 &&
                ABS_F((float)mm - (float)prev_mm) >
                    NAV_SCAN_SEGMENT_JUMP_MM)
                break;

            last_deg = deg;
            if (deg >= 70u && deg <= 110u)
                front_count++;

            if (mm < min_mm)
                min_mm = mm;
            sum += mm;
            count++;
            prev_mm = mm;
            deg += SC_STEP;
        }

        if (count > 0) {
            int span = last_deg - start_deg;
            if (span <= 0)
                span = SC_STEP;

            float center = (float)(start_deg + last_deg) * 0.5f;
            float score = (float)front_count * 120.0f
                        + (float)count * 25.0f
                        + (float)span * 4.0f
                        - ABS_F(center - 90.0f) * 2.0f
                        - (float)min_mm * 0.03f;

            if (score > best_score) {
                best_score = score;
                best.found = 1u;
                best.count = count;
                best.front_count = front_count;
                best.first_deg = start_deg;
                best.last_deg = last_deg;
                best.span_deg = span;
                best.min_mm = min_mm;
                best.avg_mm = (u16)(sum / count);
            }
        }
    }

    return best;
}

static ScanShape_t scan_shape_stats(void)
{
    ScanShape_t s = {0};
    u32 sum = 0u;

    s.min_mm = 65535u;
    s.max_mm = 0u;
    s.first_near_deg = -1;
    s.last_near_deg = -1;

    for (u8 deg = NAV_SHAPE_SCAN_MIN_DEG;
         deg <= NAV_SHAPE_SCAN_MAX_DEG;
         deg += SC_STEP) {
        u16 mm = scanner_get(deg);

        /* 9999/0/no-return is unknown, not far wall. Do not let it inflate
         * max-min spread, otherwise every real wall becomes "object". */
        if (mm == 0u || mm == 9999u || mm > NAV_SHAPE_VALID_MAX_MM)
            continue;

        s.valid_count++;
        sum += mm;

        if (mm < s.min_mm)
            s.min_mm = mm;
        if (mm > s.max_mm)
            s.max_mm = mm;

        if (mm <= NAV_SHAPE_NEAR_MM) {
            s.near_count++;
            if (deg >= 70u && deg <= 110u)
                s.front_near_count++;

            if (s.first_near_deg < 0)
                s.first_near_deg = deg;
            s.last_near_deg = deg;
        }
    }

    if (s.valid_count == 0)
        return s;

    s.valid = 1u;
    s.avg_mm = (u16)(sum / (u32)s.valid_count);
    s.spread_mm = (s.max_mm >= s.min_mm) ? (u16)(s.max_mm - s.min_mm) : 0u;
    if (s.first_near_deg >= 0 && s.last_near_deg >= s.first_near_deg)
        s.near_span_deg = s.last_near_deg - s.first_near_deg;

    return s;
}

static u8 wall_shape_confident(const ScanShape_t *s)
{
    if (!s->valid || s->valid_count < (int)NAV_SHAPE_MIN_VALID_POINTS)
        return 0u;

    if (s->near_count < (int)NAV_WALL_MIN_POINTS)
        return 0u;

    if (s->front_near_count < 4)
        return 0u;

    /* A wall/end-of-row should occupy a broad angular span in front of the
     * robot.  A small box offset to the left/right can have a small max-min
     * distance spread too, but it should not be treated as a wall.
     */
    if (s->near_span_deg < (int)NAV_WALL_MID_SPAN_DEG)
        return 0u;

    return (s->spread_mm <= NAV_SHAPE_WALL_SPREAD_MAX_MM) ? 1u : 0u;
}

static u8 object_shape_likely(const ScanShape_t *s)
{
    if (!s->valid || s->valid_count < (int)NAV_SHAPE_MIN_VALID_POINTS)
        return 0u;

    return (s->spread_mm > NAV_SHAPE_WALL_SPREAD_MAX_MM) ? 1u : 0u;
}

static u8 wall_scan_confident(void)
{
    u16 near_limit_mm =
        (u16)(NAV_OBS_LOOKAHEAD_MM + NAV_WALL_NEAR_EXTRA_MM);
    FrontBlock_t block = front_near_block(near_limit_mm);

    if (!block.found)
        return 0u;

    if (block.front_count < 4)
        return 0u;

    if (block.count < (int)NAV_WALL_MIN_POINTS)
        return 0u;

    /* Wall is not full 0..180.  It is a broad near block in the forward
     * sector.  The side edges may read far/9999 and must not turn a real wall
     * into an object. */
    if (block.span_deg >= (int)NAV_WALL_BROAD_SPAN_DEG)
        return 1u;

    if (block.span_deg >= (int)NAV_WALL_MID_SPAN_DEG &&
        block.min_mm <= (u16)(NAV_OBS_DECIDE_MM + 220u))
        return 1u;

    return 0u;
}

static Scene_t classify_scene(Gap_t *gap_out, i8 *side_out)
{
    Gap_t gap = {0};
    ScanShape_t shape = scan_shape_stats();
    u8 has_gap = Avoid_FindBestGap(&gap);
    u8 shape_ready =
        (shape.valid &&
         shape.valid_count >= (int)NAV_SHAPE_MIN_VALID_POINTS) ? 1u : 0u;
    u8 wall_block = wall_scan_confident();
    u8 wall = shape_ready ? (u8)(wall_shape_confident(&shape) || wall_block)
                          : wall_block;
    u8 object_like = object_shape_likely(&shape);

    if (gap_out)
        *gap_out = gap;

    if (wall) {
        if (side_out) *side_out = choose_open_side();
        return SCENE_WALL;
    }

    if ((shape_ready && object_like) || has_gap) {
        if (side_out) *side_out = choose_object_escape_side(&gap);
        return SCENE_OBJECT;
    }

    if (side_out) *side_out = choose_open_side();
    return SCENE_UNKNOWN;
}

static void schedule_turn_to_ex(float yaw_target, NavSt_t after_turn, u8 row_turn)
{
    pending_turn_yaw = norm_deg(yaw_target);
    pending_after_turn = after_turn;
    pending_turn_is_row = row_turn ? 1u : 0u;
    stop_motion();
    enter_state(N_TURN_WAIT);
}

static void schedule_turn_to(float yaw_target, NavSt_t after_turn)
{
    schedule_turn_to_ex(yaw_target, after_turn, 0u);
}

static void schedule_row_turn_to(float yaw_target, NavSt_t after_turn)
{
    schedule_turn_to_ex(yaw_target, after_turn, 1u);
}

static void begin_scan(void)
{
    scan_for_plan = 0u;
    clear_obstacle_plan();
    stop_motion();
    enter_state(N_BRAKE);
}

static void begin_plan_scan(void)
{
    scan_for_plan = 1u;
    planned_front_mm = front_hold_mm;
    if (planned_front_mm == 0u || planned_front_mm == 9999u)
        planned_front_mm = NAV_OBS_LOOKAHEAD_MM;
    if (planned_front_mm < NAV_OBS_TURN_MM)
        planned_front_mm = NAV_OBS_TURN_MM;

    planned_x = pose.x;
    planned_y = pose.y;
    planned_run_m =
        ((float)(planned_front_mm - NAV_OBS_TURN_MM)) * 0.001f;

    stop_motion();
    enter_state(N_BRAKE);
}

static void begin_stuck_back(void)
{
    HeadingHold_SetTarget(yaw);
    stop_motion();
    enter_state(N_STUCK_BACK);
}

static void begin_wall_row_change(i8 side)
{
    object_avoid_active = 0u;

    if ((u8)(nav.row + 1u) >= NAV_MAX_ROWS) {
        enter_state(N_DONE);
        return;
    }

    wall_row_active = 1u;

    /*
     * Once a wall is confirmed, row-change has priority:
     * finish 90 deg turn -> cross to the new row -> turn into the row.
     * Any object seen by VL53 while the robot is rotating/crossing belongs to
     * the next scene and must not interrupt this wall maneuver.
     */
    clear_obstacle_plan();
    scanner_unlock();
    if (scanner_mode() == SC_WIDE)
        scanner_wide_consume();
    scanner_pause(1u);

    row_shift_side = (side >= 0) ? 1 : -1;
    row_next_yaw = norm_deg(nav.lane_yaw + 180.0f);
    row_shift_target_m = NAV_WALL_ROW_M;

    schedule_row_turn_to(nav.lane_yaw +
                         yaw_delta_for_side(row_shift_side,
                                            NAV_AVOID_ANGLE_DEG),
                         N_ROW_CROSS);
}

static void begin_length_row_change(void)
{
    begin_wall_row_change(nav.dir);
    row_shift_target_m = NAV_ROW_M;
}

static void begin_object_avoid(i8 side)
{
    object_avoid_active = 1u;
    wall_row_active = 0u;

    avoid_resume_yaw = nav.lane_yaw;
    avoid_resume_x0 = nav.x0;
    avoid_resume_y0 = nav.y0;
    avoid_side = (side >= 0) ? 1 : -1;

    /* Side-follow object avoidance:
     * turn 90 deg, lock the VL53 toward the object side, then use side
     * detection to decide whether to follow the object's edge or fall back to
     * a fixed encoder distance.  This keeps the robot moving even when the
     * side measurement is missing.
     */
    avoid_side_seen = 0u;
    avoid_side_lost_count = 0u;
    scanner_lock_angle(object_side_servo_deg());

    schedule_turn_to(avoid_resume_yaw +
                     yaw_delta_for_side(avoid_side,
                                        NAV_AVOID_ANGLE_DEG),
                     N_AVOID_OFFSET);
}

static void execute_scene_decision(Scene_t scene, i8 side)
{
    float obj_offset_m = planned_avoid_offset_m;
    float obj_pass_m = planned_avoid_pass_m;

    clear_obstacle_plan();

    if (scene == SCENE_WALL) {
        /* Wall/end-of-row must follow the zigzag lane order, not the
         * currently "more open" scan side.  nav.dir is updated after each row
         * cross, so the wall turn alternates left/right on each row. */
        begin_wall_row_change(nav.dir);
    } else if (scene == SCENE_OBJECT) {
        avoid_offset_m =
            limit(obj_offset_m,
                  NAV_AVOID_OFFSET_MIN_M,
                  NAV_AVOID_OFFSET_MAX_M);
        avoid_pass_m =
            limit(obj_pass_m,
                  NAV_AVOID_PASS_MIN_M,
                  NAV_AVOID_PASS_MAX_M);
        begin_object_avoid(side);
    } else {
        begin_stuck_back();
    }
}

static void plan_from_scan(void)
{
    Gap_t gap;
    i8 side = nav.dir;
    Scene_t scene = classify_scene(&gap, &side);

    planned_scene = scene;
    planned_ready = 1u;

    if (scene == SCENE_OBJECT) {
        ObjectMeasure_t obj = estimate_object_from_scan();
        if (obj.found) {
            set_planned_avoid_from_width(obj.width_m);

            i8 object_side = side_from_object_position(&obj);
            if (object_side != 0)
                side = object_side;
        } else {
            set_planned_avoid_from_width(NAV_AVOID_OFFSET_M);
            planned_avoid_offset_m = NAV_AVOID_OFFSET_M;
            planned_avoid_pass_m = NAV_AVOID_PASS_M;
        }
    } else {
        planned_object_width_m = 0.0f;
        planned_avoid_offset_m = NAV_AVOID_OFFSET_M;
        planned_avoid_pass_m = NAV_AVOID_PASS_M;
    }

    planned_side = side;
    scanner_wide_consume();
}

static void fallback_object_plan_from_partial_scan(void)
{
    Gap_t gap = {0};
    i8 side = nav.dir;

    if (Avoid_FindBestGap(&gap)) {
        side = choose_object_escape_side(&gap);
    } else {
        side = choose_open_side();
    }

    planned_scene = SCENE_OBJECT;
    planned_ready = 1u;

    ObjectMeasure_t obj = estimate_object_from_scan();
    if (obj.found) {
        set_planned_avoid_from_width(obj.width_m);

        i8 object_side = side_from_object_position(&obj);
        if (object_side != 0)
            side = object_side;
    } else {
        /* Conservative default for a small/medium box when the scan did not
         * produce a clean object segment.  This prevents the robot from
         * waiting forever in front of an obstacle. */
        set_planned_avoid_from_width(0.35f);
    }

    planned_side = side;

    if (scanner_mode() == SC_WIDE)
        scanner_wide_consume();
}

static void execute_planned_or_object_fallback(void);

static void execute_planned_or_object_fallback(void)
{
    Scene_t scene = planned_scene;
    i8 side = planned_side;

    /* At the 20 cm action point the robot must not sit there scanning
     * forever.  If the scan is not clear enough to classify, still treat it
     * as an obstacle and choose the more open side, then rotate 90 degrees.
     */
    if (scene == SCENE_UNKNOWN) {
        side = choose_open_side();
        scene = SCENE_OBJECT;
    }

    execute_scene_decision(scene, side);
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
    reset_stuck_watch();
    clear_obstacle_plan();
    mission_set_line(yaw, pose.x, pose.y);
    HeadingHold_Enable(1u);
}

void nav_task(void)
{
    u32 now = HAL_GetTick();

    switch (nav.st) {
    case N_BOOT:
        stop_motion();
        if ((u32)(now - nav.t0) >= NAV_BOOT_DELAY_MS) {
            mission_set_line(yaw, pose.x, pose.y);
            enter_state(N_FWD);
        }
        break;

    case N_FWD: {
        u16 front;
        if (take_new_front(&front))
            front_hold_mm = front;

        u8 plan_turn_reached =
            (planned_ready && planned_distance_done() >= planned_run_m)
                ? 1u : 0u;

        if (!planned_ready &&
            front_hold_mm > (NAV_OBS_LOOKAHEAD_MM + 80u)) {
            clear_obstacle_plan();
            if (scanner_mode() == SC_WIDE)
                scanner_wide_consume();
        } else if (front_hold_mm <= NAV_OBS_LOOKAHEAD_MM &&
                   planned_ready == 0u) {
            begin_plan_scan();
            break;
        }

        if (lane_progress() >= NAV_ROW_LENGTH_M) {
            begin_length_row_change();
        } else if (plan_turn_reached ||
                   front_hold_mm <= NAV_OBS_TURN_MM) {
            if (planned_ready) {
                execute_planned_or_object_fallback();
            } else {
                begin_scan();
            }
        } else {
            drive_lane(speed_for_front(front_hold_mm));
            if (stuck_detected(now, g_sp_v))
                begin_stuck_back();
        }
        break;
    }

    case N_BRAKE:
        stop_motion();
        if ((u32)(now - nav.t0) < NAV_BRAKE_MS)
            break;

        if (scan_for_plan) {
            u8 timed_out =
                ((u32)(now - nav.t0) >= NAV_SCAN_TIMEOUT_MS) ? 1u : 0u;

            if (scanner_wide_ready() || timed_out) {
                scan_for_plan = 0u;

                if (scanner_wide_ready())
                    plan_from_scan();
                else
                    fallback_object_plan_from_partial_scan();

                if (front_hold_mm <= NAV_OBS_TURN_MM) {
                    execute_planned_or_object_fallback();
                } else {
                    resume_forward_keep_plan();
                }
            }
        } else {
            u8 timed_out =
                ((u32)(now - nav.t0) >= NAV_SCAN_TIMEOUT_MS) ? 1u : 0u;
            if (wide_scan_usable() || timed_out) {
                if (timed_out && !wide_scan_usable())
                    fallback_object_plan_from_partial_scan();
                else
                    plan_from_scan();
                execute_planned_or_object_fallback();
            }
        }
        break;

    case N_TURN_WAIT:
        stop_motion();
        if ((u32)(now - nav.t0) >= NAV_STOP_SETTLE_MS) {
            turn_start_to(pending_turn_yaw);
            enter_state(N_TURN_ACTIVE);
        }
        break;

    case N_TURN_ACTIVE: {
        float err = fabsf(angle_diff(pending_turn_yaw, yaw));
        u8 force_row =
            (pending_turn_is_row &&
             (u32)(now - nav.t0) >= NAV_ROW_TURN_FORCE_MS &&
             err <= NAV_ROW_TURN_FORCE_ERR_DEG) ? 1u : 0u;
        u8 accept_row =
            (pending_turn_is_row &&
             err <= NAV_ROW_TURN_ACCEPT_ERR_DEG) ? 1u : 0u;
        u8 accept_avoid =
            (!pending_turn_is_row &&
             (pending_after_turn == N_AVOID_OFFSET ||
              pending_after_turn == N_AVOID_PASS ||
              pending_after_turn == N_AVOID_REJOIN) &&
             err <= NAV_AVOID_TURN_ACCEPT_ERR_DEG) ? 1u : 0u;
        u8 force_avoid =
            (!pending_turn_is_row &&
             (pending_after_turn == N_AVOID_OFFSET ||
              pending_after_turn == N_AVOID_PASS ||
              pending_after_turn == N_AVOID_REJOIN) &&
             (u32)(now - nav.t0) >= NAV_AVOID_TURN_FORCE_MS &&
             err <= NAV_AVOID_TURN_FORCE_ERR_DEG) ? 1u : 0u;

        if ((turn_done() && !turn.timeout) ||
            accept_row ||
            accept_avoid ||
            force_row ||
            force_avoid) {
            stop_motion();
            turn.state = TR_DONE;
            turn.done = 1u;
            turn.timeout = 0u;
            HeadingHold_SetTarget(pending_turn_yaw);
            enter_state(pending_after_turn);
        } else if (turn_done() && turn.timeout) {
            if (object_avoid_active) {
                stop_motion();
                turn.state = TR_DONE;
                turn.done = 1u;
                turn.timeout = 0u;
                HeadingHold_SetTarget(pending_turn_yaw);
                enter_state(pending_after_turn);
            } else if (wall_row_active && pending_turn_is_row) {
                stop_motion();
                turn.state = TR_DONE;
                turn.done = 1u;
                turn.timeout = 0u;
                HeadingHold_SetTarget(pending_turn_yaw);
                enter_state(pending_after_turn);
            } else {
                begin_stuck_back();
            }
        }
        break;
    }

    case N_ROW_CROSS:
        drive_heading(NAV_SPEED * 0.8f);
        if (row_cross_encoder_distance() >= row_shift_target_m ||
            (wall_row_active &&
             (u32)(now - nav.t0) >= NAV_WALL_ROW_CROSS_TIMEOUT_MS)) {
            nav.row++;
            nav.dir = (i8)(-row_shift_side);
            mission_set_line(row_next_yaw, pose.x, pose.y);
            schedule_row_turn_to(nav.lane_yaw, N_FWD);
        } else if (!wall_row_active &&
                   (u32)(now - nav.t0) >= NAV_ROW_CROSS_STUCK_GRACE_MS &&
                   stuck_detected(now, g_sp_v)) {
            begin_stuck_back();
        }
        break;

    case N_AVOID_OFFSET: {
        float travelled = encoder_distance_from_mark();
        u8 ignore_front =
            (travelled < NAV_AVOID_FRONT_IGNORE_M ||
             (u32)(now - nav.t0) < NAV_AVOID_FRONT_IGNORE_MS) ? 1u : 0u;
        float offset_target =
            limit(avoid_offset_m,
                  NAV_AVOID_OFFSET_MIN_M,
                  NAV_AVOID_OFFSET_MAX_M);

        drive_heading(NAV_AVOID_SPEED);

        /* After the robot has turned 90 deg toward the open side, this state
         * must only move across the obstacle width.  Do not let side VL53
         * readings decide another turn here; they are handled in N_AVOID_PASS
         * for the obstacle length.  This prevents left-right oscillation.
         */
        if (travelled >= offset_target) {
            avoid_actual_offset_m = travelled;
            schedule_turn_to(avoid_resume_yaw, N_AVOID_PASS);
        } else if (!object_avoid_active &&
                   !ignore_front &&
                   stuck_detected(now, g_sp_v)) {
            begin_stuck_back();
        }
        break;
    }

    case N_AVOID_PASS: {
        float travelled = encoder_distance_from_mark();
        u8 side_seen_now = side_sensor_sees_object();
        drive_heading(NAV_AVOID_SPEED);

        if (avoid_pass_phase == 0u) {
            if (side_seen_now) {
                avoid_side_seen = 1u;
                avoid_side_lost_count = 0u;
            } else if (avoid_side_seen) {
                if (avoid_side_lost_count < 255u)
                    avoid_side_lost_count++;
            }

            /* At the beginning of the pass, the side VL53 may not see the
             * object yet: that means the robot is not alongside the object
             * yet, not that it has already passed it.  Keep driving forward
             * until the side VL53 sees the object, then keep driving while it
             * still sees it.  Only after the object has been seen once and
             * then lost do we add robot-length clearance and return to the old
             * line.  Do not finish just because an estimated pass distance is
             * reached: for object length, the side VL53 is the main signal.
             */
            if (avoid_side_seen &&
                avoid_side_lost_count >= NAV_SIDE_LOST_COUNT &&
                travelled >= NAV_AVOID_PASS_MIN_RUN_M) {
                avoid_pass_phase = 1u;
                mark_position();
                break;
            }

            if (travelled >= NAV_PASS_HARD_MAX_M) {
                avoid_pass_phase = 1u;
                mark_position();
                break;
            }

            if (!object_avoid_active &&
                travelled > NAV_AVOID_FRONT_IGNORE_M &&
                stuck_detected(now, g_sp_v)) {
                begin_stuck_back();
            }
            break;
        }

        if (encoder_distance_from_mark() >= NAV_AFTER_OBJECT_CLEAR_M) {
            schedule_turn_to(avoid_resume_yaw -
                             yaw_delta_for_side(avoid_side,
                                                NAV_AVOID_ANGLE_DEG),
                             N_AVOID_REJOIN);
        } else if (!object_avoid_active &&
                   stuck_detected(now, g_sp_v)) {
            begin_stuck_back();
        }
        break;
    }

    case N_AVOID_REJOIN: {
        float e = cross_error_for(avoid_resume_yaw,
                                  avoid_resume_x0,
                                  avoid_resume_y0);
        float travelled = encoder_distance_from_mark();
        float e_abs = fabsf(e);
        u8 tight_lane = (fabsf(e) <= NAV_REJOIN_ERR_M);
        u8 crossed_lane =
            (fabsf(rejoin_start_error) > NAV_AVOID_REJOIN_LEAD_M &&
             (rejoin_start_error * e) <= 0.0f &&
             e_abs <= NAV_REJOIN_CROSS_ERR_M);
        float max_rejoin_m = avoid_actual_offset_m + NAV_REJOIN_MAX_OVER_M;
        float rejoin_speed =
            (e_abs <= 0.12f) ? (NAV_SPEED * 0.30f) : (NAV_SPEED * 0.45f);

        drive_heading(rejoin_speed);

        if (travelled >= NAV_REJOIN_MIN_M &&
            (tight_lane || crossed_lane)) {
            mission_set_line(avoid_resume_yaw,
                             avoid_resume_x0,
                             avoid_resume_y0);
            schedule_turn_to(nav.lane_yaw, N_FWD);
        } else if (travelled >= max_rejoin_m) {
            /* End the rejoin leg without abandoning the original line.
             * N_FWD keeps the old lane reference and will continue trimming
             * back toward it instead of creating a new parallel line here.
             */
            schedule_turn_to(nav.lane_yaw, N_FWD);
        } else if (!object_avoid_active &&
                   stuck_detected(now, g_sp_v)) {
            begin_stuck_back();
        }
        break;
    }

    case N_STUCK_BACK:
        drive_heading(-NAV_STUCK_BACK_SPEED);
        if (distance_from_mark() >= NAV_STUCK_BACK_M ||
            (u32)(now - nav.t0) >= NAV_STUCK_BACK_TIMEOUT_MS) {
            begin_scan();
        }
        break;

    case N_DONE:
    default:
        stop_motion();
        nav.done = 1u;
        break;
    }
}

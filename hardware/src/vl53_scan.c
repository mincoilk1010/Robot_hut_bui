#include "vl53_scan.h"
#include "control.h"
#include <math.h>

Scanner_t sc = {0};
Vl53MapCell_t vl53_map[SC_MAP_BINS] = {0};
u16 d = 0;

static void _svo(u8 deg)
{
    if (deg > 180u) deg = 180u;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,
                          500u + (u32)deg * 2000u / 180u);
}

void scanner_map_clear(void)
{
    for (u8 i = 0u; i < SC_MAP_BINS; i++) {
        vl53_map[i].raw_mm = 9999u;
        vl53_map[i].filt_mm = 9999u;
        vl53_map[i].stamp = 0u;
        vl53_map[i].valid = 0u;
        vl53_map[i].conf = 0u;
    }
}

static void scanner_map_update(u8 deg, u16 mm)
{
    u8 idx = deg / SC_STEP;
    if (idx >= SC_MAP_BINS)
        return;

    if (mm != 0u && mm != 9999u) {
        vl53_map[idx].raw_mm = mm;

        if (!vl53_map[idx].valid || vl53_map[idx].filt_mm == 9999u) {
            vl53_map[idx].filt_mm = mm;
        } else {
            vl53_map[idx].filt_mm =
                (u16)(((u32)vl53_map[idx].filt_mm * 7u +
                       (u32)mm * 3u) / 10u);
        }

        vl53_map[idx].stamp = HAL_GetTick();
        vl53_map[idx].valid = 1u;
        if (vl53_map[idx].conf < SC_MAP_CONF_MAX)
            vl53_map[idx].conf++;
    } else {
        vl53_map[idx].raw_mm = 9999u;
        if (vl53_map[idx].conf > 0u)
            vl53_map[idx].conf--;
        if (vl53_map[idx].conf == 0u)
            vl53_map[idx].valid = 0u;
    }
}

/* ★FIX: không còn reset sc.angle — chỉ đổi vùng quét (amin/amax).
 * Servo tiếp tục từ góc hiện tại, không giật/nhảy cóc.            */
static void _enter_narrow(void)
{
    sc.mode = SC_NARROW;
    sc.amin = SC_N_MIN;
    sc.amax = SC_N_MAX;
    sc.wide_hold = 0u;
    sc.wide_ready = 0u;
    sc.front_mask = 0u;
    /* Nếu góc hiện tại đang ngoài vùng hẹp mới (vd đang ở 20° hay 170°
     * từ lần quét rộng trước), KHÔNG ép về amin — cứ để vòng lặp bước
     * tới (theo sc.dir hiện tại) tự nhiên đưa nó vào trong [amin,amax]
     * ở các lần SC_READ tiếp theo, tránh giật servo.                 */
}
static void _enter_wide(void)
{
    sc.mode = SC_WIDE;
    sc.amin = SC_W_MIN;
    sc.amax = SC_W_MAX;
    sc.clear_cnt = 0;
    sc.wide_ready = 0u;
    sc.front_mask = 0u;
    /* ★FIX: dữ liệu wide-scan cũ (từ lần WIDE trước, có thể ở tình
     * huống hoàn toàn khác) phải bị xoá, không để Avoid_FindBestGap()
     * lượm dữ liệu rác → chọn nhầm hướng né. Đặt lại 9999 (coi như
     * "chưa biết") để các hàm né phải đợi servo quét MỚI mới quyết
     * định, tránh xe quay nhầm hướng rồi cứ thế lặp lại.            */
    sc.wide_valid_count = 0;
    for (int i = 0; i < 37; i++) {
        sc.data[i] = 9999u;
        sc.stamp[i] = 0u;
        sc.wide_valid[i] = 0u;
    }
}

void scanner_request_wide(void)
{
    /* A cliff/end-of-row decision also needs a fresh 0..180 degree scan,
     * even when the front range sensor did not trigger wide mode itself. */
    if (sc.mode != SC_WIDE)
        _enter_wide();
    sc.wide_hold = 1u;
}

void scanner_wide_consume(void)
{
    sc.wide_ready = 0u;
    sc.wide_hold = 0u;
    _enter_narrow();
}

void scanner_lock_angle(u8 deg)
{
    if (deg > 180u) deg = 180u;
    sc.locked = 1u;
    sc.lock_angle = deg;
    sc.angle = deg;
    _svo(deg);
    sc.t_servo = HAL_GetTick();
    sc.lock_mm = 9999u;
    sc.lock_stamp = 0u;
    sc.lock_seq = 0u;
    sc.wide_ready = 0u;
    sc.wide_hold = 0u;
    sc.front_mask = 0u;
    sc.mode = SC_NARROW;
    sc.state = SC_WAIT;
}

void scanner_unlock(void)
{
    sc.locked = 0u;
    sc.lock_mm = 9999u;
    sc.lock_stamp = 0u;
    _enter_narrow();

    if (sc.angle < sc.amin) {
        sc.angle = sc.amin;
        sc.dir = 1;
    } else if (sc.angle > sc.amax) {
        sc.angle = sc.amax;
        sc.dir = -1;
    }

    _svo(sc.angle);
    sc.t_servo = HAL_GetTick();
    sc.state = SC_MOVE;
}

void scanner_init(void)
{
    scanner_map_clear();

    for (int i = 0; i < 37; i++) {
        sc.data[i] = 9999u;
        sc.stamp[i] = 0u;
        sc.wide_valid[i] = 0u;
    }
    sc.mode  = SC_NARROW;
    sc.amin  = SC_N_MIN;
    sc.amax  = SC_N_MAX;
    sc.angle = SC_N_MIN;
    sc.dir   = 1;            /* ★ bắt đầu quét lên (amin→amax) */
    sc.done  = 0;
    sc.cycle = 0;
    sc.wide_valid_count = 0;
    sc.wide_ready = 0u;
    sc.wide_hold = 0u;
    sc.front_mask = 0u;
    sc.front_mm = 9999u;
    sc.front_stamp = 0u;
    sc.front_seq = 0u;
    sc.locked = 0u;
    sc.lock_angle = 90u;
    sc.lock_mm = 9999u;
    sc.lock_stamp = 0u;
    sc.lock_seq = 0u;
    _svo(sc.angle);
    HAL_Delay(50);
}

void scanner_task(void)
{
    uint32_t now = HAL_GetTick();

    if (sc.locked) {
        switch (sc.state) {
        case SC_IDLE:
        case SC_MOVE:
            sc.angle = sc.lock_angle;
            _svo(sc.lock_angle);
            sc.t_servo = now;
            sc.state = SC_WAIT;
            break;

        case SC_WAIT:
            if (now - sc.t_servo >= SC_WAIT_MS)
                sc.state = SC_READ;
            break;

        case SC_READ:
            d = (uint16_t)readRangeContinuousMillimeters(0);
            if (d == 0 || d > 2000u) d = 9999u;
            scanner_map_update(sc.lock_angle, d);
            sc.lock_mm = d;
            sc.lock_stamp = now;
            sc.lock_seq++;
            sc.state = SC_MOVE;
            break;
        }
        return;
    }

    switch (sc.state) {

    case SC_IDLE:
        sc.state = SC_MOVE;
        break;

    case SC_MOVE:
        _svo(sc.angle);
        sc.t_servo = now;
        sc.state   = SC_WAIT;
        break;

    case SC_WAIT:
        if (now - sc.t_servo >= SC_WAIT_MS) sc.state = SC_READ;
        break;

    case SC_READ: {
        d = (uint16_t)readRangeContinuousMillimeters(0);
        if (d == 0 || d > 2000u) d = 9999u;
        scanner_map_update(sc.angle, d);

        /* ── Chuyển NARROW→WIDE khi gặp vật gần ── chỉ đổi vùng quét,
         * KHÔNG đổi sc.angle/sc.dir → servo không bị giật.          */
        /* nav_task() owns wide-scan decisions.  If the scanner enters WIDE
         * by itself while the robot is doing a wall row-change, the servo can
         * swing back to the side wall and the nav state can be confused.
         */

        /* Save the latest sample for the current scan mode. */
        u8 idx = sc.angle / SC_STEP;
        if (idx < 37u) {
            sc.data[idx] = d;
            sc.stamp[idx] = now;
            if (sc.mode == SC_WIDE && !sc.wide_valid[idx]) {
                sc.wide_valid[idx] = 1u;
                sc.wide_valid_count++;
            }

            /* Publish a front result only after 85/90/95 degrees were all
             * measured in the current pass. nav_task() can then count real
             * frames instead of counting the same cached value every 20 ms. */
            if (sc.angle == 85u) sc.front_mask |= 0x01u;
            if (sc.angle == 90u) sc.front_mask |= 0x02u;
            if (sc.angle == 95u) sc.front_mask |= 0x04u;
            if (sc.front_mask == 0x07u) {
                u16 d1 = sc.data[85u / SC_STEP];
                u16 d2 = sc.data[90u / SC_STEP];
                u16 d3 = sc.data[95u / SC_STEP];
                if (d1 > d2) { u16 t = d1; d1 = d2; d2 = t; }
                if (d2 > d3) { u16 t = d2; d2 = d3; d3 = t; }
                if (d1 > d2) { u16 t = d1; d1 = d2; d2 = t; }
                sc.front_mm = d2;
                sc.front_stamp = now;
                sc.front_seq++;
                sc.front_mask = 0u;
            }
        }

        if (sc.mode == SC_WIDE && sc.wide_valid_count >= 37u)
            sc.wide_ready = 1u;
        

        /* ── Quay lại NARROW sau 5 lần liên tục thấy thoáng ── */
        if (sc.mode == SC_WIDE && sc.wide_ready && !sc.wide_hold &&
            sc.angle >= SC_N_MIN && sc.angle <= SC_N_MAX) {
            if (d > SC_CLEAR_MM) sc.clear_cnt++;
            else                 sc.clear_cnt = 0;
            if (sc.clear_cnt >= 5u) _enter_narrow();
        }

        /* ════════════════════════════════════════════════════════
         * ★FIX CHÍNH: quét ping-pong, KHÔNG snap về 90°/amin.
         * Tăng dần tới amax → đảo hướng → giảm dần tới amin → đảo
         * hướng → lặp lại. HÀNH VI GIỐNG NHAU bất kể đang ở mode
         * nào (NARROW hay WIDE), bất kể có vật hay không — chỉ phụ
         * thuộc việc chạm biên amin/amax của vùng quét hiện tại.
         * ════════════════════════════════════════════════════════ */
        if (sc.dir > 0) {
            if (sc.angle < sc.amax) {
                sc.angle += SC_STEP;
            } else {
                sc.dir = -1;                 /* đảo hướng, KHÔNG nhảy cóc */
                if (sc.angle > sc.amin) sc.angle -= SC_STEP;
                sc.done = 1; sc.cycle++;      /* vừa quét hết 1 lượt lên */
            }
        } else {
            if (sc.angle > sc.amin) {
                sc.angle -= SC_STEP;
            } else {
                sc.dir = 1;
                if (sc.angle < sc.amax) sc.angle += SC_STEP;
                sc.done = 1; sc.cycle++;      /* vừa quét hết 1 lượt xuống */
            }
        }
        sc.state = SC_MOVE;
        break;
    }
    }
}

u16 scanner_get(u8 deg)
{
    u8 i = deg / SC_STEP;
    if (i >= 37u || sc.stamp[i] == 0u) return 9999u;
    if ((u32)(HAL_GetTick() - sc.stamp[i]) > SC_SAMPLE_MAX_AGE_MS) return 9999u;
    return sc.data[i];
}

u16 scanner_map_get(u8 deg)
{
    u8 i = deg / SC_STEP;
    if (i >= SC_MAP_BINS || !vl53_map[i].valid || vl53_map[i].stamp == 0u)
        return 9999u;
    if ((u32)(HAL_GetTick() - vl53_map[i].stamp) > SC_MAP_MAX_AGE_MS)
        return 9999u;
    return vl53_map[i].filt_mm;
}

u8 scanner_map_conf(u8 deg)
{
    u8 i = deg / SC_STEP;
    if (i >= SC_MAP_BINS)
        return 0u;
    if (vl53_map[i].stamp == 0u ||
        (u32)(HAL_GetTick() - vl53_map[i].stamp) > SC_MAP_MAX_AGE_MS)
        return 0u;
    return vl53_map[i].conf;
}

u16 scanner_front(void) {
	u32 max_age = (sc.mode == SC_WIDE) ? SC_FRONT_WIDE_MAX_AGE_MS
	                                    : SC_FRONT_NARROW_MAX_AGE_MS;
	if (sc.front_stamp == 0u ||
	    (u32)(HAL_GetTick() - sc.front_stamp) > max_age)
		return 9999u;
	return sc.front_mm;
}

u32 scanner_front_seq(void) { return sc.front_seq; }

u16 scanner_locked_mm(void)
{
    if (!sc.locked || sc.lock_stamp == 0u ||
        (u32)(HAL_GetTick() - sc.lock_stamp) > SC_LOCK_MAX_AGE_MS)
        return 9999u;
    return sc.lock_mm;
}

u32 scanner_locked_seq(void) { return sc.lock_seq; }

u8 scanner_has_obs(void)
{
    for (u8 a = SC_N_MIN; a <= SC_N_MAX; a += SC_STEP)
        if (scanner_get(a) < SC_OBS_MM) return 1;
    return 0;
}
ScMode_t scanner_mode(void) { return sc.mode; }

/* Ready only after every wide-scan bin has a fresh sample. */
u8 scanner_wide_ready(void)
{
    return (sc.mode == SC_WIDE) && sc.wide_ready;
}

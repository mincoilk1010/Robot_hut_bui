#include "avoid.h"
#include "vl53_scan.h"
#include <math.h>

uint8_t Avoid_FindBestGap(Gap_t *out)
{
    out->found = 0u;
    Gap_t best = {0};
    float best_score = -1.0f;

    int n = (SC_W_MAX - SC_W_MIN) / SC_STEP + 1;
    int i = 0;

    while (i < n) {
        if (sc.data[i] < GAP_CLEAR_MM) {
            i++;
            continue;
        }

        int start = i;
        long sum = 0;
        int cnt = 0;
        int unknown_cnt = 0;
        int real_cnt = 0;

        while (i < n && sc.data[i] >= GAP_CLEAR_MM) {
            if (sc.data[i] == 9999u) {
                sum += GAP_UNKNOWN_MM;
                unknown_cnt++;
            } else {
                sum += sc.data[i];
                real_cnt++;
            }
            cnt++;
            i++;
        }

        int end = i - 1;
        if (cnt == 0) {
            i++;
            continue;
        }

        /* Do not let unmeasured 9999 bins become a fake escape path.  A valid
         * gap must contain some real VL53 samples; unknown bins may extend a
         * real opening, but must not dominate it. */
        if (real_cnt < (int)GAP_REAL_MIN_POINTS)
            continue;
        if (unknown_cnt == cnt && cnt < GAP_UNKNOWN_MIN_SPAN_POINTS)
            continue;
        if (unknown_cnt > real_cnt * (int)GAP_UNKNOWN_MAX_RATIO)
            continue;

        uint16_t avg_mm = (uint16_t)(sum / cnt);
        int span_n = end - start + 1;
        float span_deg = (float)((span_n - 1) * SC_STEP);
        if (span_n == 1)
            span_deg = (float)SC_STEP;

        float span_rad = DEG2RAD(span_deg);
        float avg_d_m = (float)avg_mm / 1000.0f;
        float width_m = 2.0f * avg_d_m * sinf(span_rad * 0.5f);

        /* Gap must be wider than the robot plus safety margin.
         * Do not relax this at long distance: a narrow far corridor will only
         * become risky when the robot reaches it. */
        if (width_m < MIN_GAP_M)
            continue;

        uint8_t center_deg =
            (uint8_t)(SC_W_MIN + (start + end) * SC_STEP / 2);

        float dev_penalty = ABS_F((float)center_deg - 90.0f) * 2.0f;
        float unknown_penalty =
            (float)unknown_cnt * GAP_UNKNOWN_PENALTY_MM;
        float score = (float)avg_mm - dev_penalty - unknown_penalty;

        if (score > best_score) {
            best_score = score;
            best.found = 1u;
            best.center_deg = center_deg;
            best.width_m = width_m;
            best.avg_mm = avg_mm;
            best.redirect_deg = (float)center_deg - 90.0f;
        }
    }

    *out = best;
    return best.found;
}

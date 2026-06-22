#include "avoid.h"
#include "vl53_scan.h"
#include <math.h>

uint8_t Avoid_FindBestGap(Gap_t *out)
{
    out->found = 0;
    Gap_t best = {0};
    float best_score = -1.0f;
 
    int n = (SC_W_MAX - SC_W_MIN) / SC_STEP + 1;  /* 37 điểm */
    int i = 0;
 
    while (i < n) {
        /* Tìm điểm bắt đầu 1 đoạn "thoáng" */
        if (sc.data[i] < GAP_CLEAR_MM) { i++; continue; }
 
        int start = i;
        long sum  = 0;
        int  cnt  = 0;
        while (i < n && sc.data[i] >= GAP_CLEAR_MM &&
               sc.data[i] != 9999u) {
            sum += sc.data[i]; cnt++; i++;
        }
        /* Cho phép đoạn toàn 9999 (không có gì) → coi avg = 1500mm */
        if (cnt == 0) {
            /* đoạn toàn no-echo: dùng giá trị mặc định an toàn */
            int j = start;
            while (j < n && (sc.data[j] >= GAP_CLEAR_MM)) j++;
            cnt = j - start;
            sum = (long)cnt * 1500;
            i = j;
        }
        int end = i - 1;
        if (cnt == 0) continue;
 
        uint16_t avg_mm  = (uint16_t)(sum / cnt);
        int      span_n  = end - start + 1;          /* số điểm */
        float    span_deg= (float)((span_n-1) * SC_STEP);
        if (span_n == 1) span_deg = (float)SC_STEP;   /* tối thiểu 1 bước */
 
        /* Chord width = 2 * d * sin(span/2) */
        float span_rad = DEG2RAD(span_deg);
        float avg_d_m  = (float)avg_mm / 1000.0f;
        float width_m  = 2.0f * avg_d_m * sinf(span_rad * 0.5f);
 
        /* ★ Kiểm tra xe có lọt qua được không ── */
        if (width_m < MIN_GAP_M) continue;   /* quá hẹp, bỏ qua */
 
        uint8_t center_deg = (uint8_t)(SC_W_MIN + (start+end)*SC_STEP/2);
 
        /* Điểm số: ưu tiên avg_mm lớn (thoáng hơn),
         * trừ nhẹ theo độ lệch khỏi 90° (ít phải xoay) */
        float dev_penalty = ABS_F((float)center_deg - 90.0f) * 2.0f; /* mm-equivalent */
        float score = (float)avg_mm - dev_penalty;
 
        if (score > best_score) {
            best_score = score;
            best.found        = 1;
            best.center_deg   = center_deg;
            best.width_m      = width_m;
            best.avg_mm       = avg_mm;
            best.redirect_deg = (float)center_deg - 90.0f;
        }
    }
 
    *out = best;
    return best.found;
}
 

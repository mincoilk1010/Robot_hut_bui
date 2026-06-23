#include "avoid.h"
#include "vl53_scan.h"
#include <math.h>

uint8_t Avoid_FindBestGap(Gap_t *out)
{
    out->found = 0;
    Gap_t best = {0};
    float best_score = -1.0f;
 
    int n = (SC_W_MAX - SC_W_MIN) / SC_STEP + 1;  /* 37 điểm quét */
    int i = 0;
 
    while (i < n) {
        /* Tìm điểm bắt đầu của 1 vùng "thoáng" */
        if (sc.data[i] < GAP_CLEAR_MM) { i++; continue; }
 
        int start = i;
        long sum  = 0;
        int  cnt  = 0;

        /* ĐÃ SỬA: Gộp chung cả điểm thường và điểm 9999u (no-echo) vào cùng 1 dải thoáng
         * không để góc quét bị xé nhỏ ra nữa */
        while (i < n && sc.data[i] >= GAP_CLEAR_MM) {
            if (sc.data[i] == 9999u) {
                sum += 1500; /* Nếu không có phản hồi, coi như thoáng cách 1.5 mét */
            } else {
                sum += sc.data[i];
            }
            cnt++;
            i++;
        }

        int end = i - 1;
        if (cnt == 0) continue;
 
        uint16_t avg_mm  = (uint16_t)(sum / cnt);
        int      span_n  = end - start + 1;          /* số điểm quét liên tục */
        float    span_deg= (float)((span_n - 1) * SC_STEP);
        if (span_n == 1) span_deg = (float)SC_STEP;   /* tối thiểu 1 bước quét */
 
        /* Tính toán bề rộng hình học của khe hở (Chord width) */
        float span_rad = DEG2RAD(span_deg);
        float avg_d_m  = (float)avg_mm / 1000.0f;
        float width_m  = 2.0f * avg_d_m * sinf(span_rad * 0.5f);
 
        /* ĐÃ SỬA: Nới lỏng điều kiện lọc khoảng trống dựa theo khoảng cách (Dynamic Filter) */
        float allowed_min_width = MIN_GAP_M; /* mặc định là 0.30m (30cm) khi ở gần */

        /* Nếu khoảng trống ở xa (> 600mm), robot chỉ cần hướng về đó chứ chưa cần lọt xe ngay,
         * càng đi lại gần thì góc mở (width_m) sẽ tự động rộng ra. Hạ tiêu chuẩn xuống 18cm ở xa. */
        if (avg_mm > 600u) {
            allowed_min_width = 0.18f;
        }

        /* Kiểm tra xe có hướng vào được không */
        if (width_m < allowed_min_width) continue;   /* Quá hẹp thực sự mới bỏ qua */
 
        uint8_t center_deg = (uint8_t)(SC_W_MIN + (start + end) * SC_STEP / 2);
 
        /* Tính điểm số: Ưu tiên khoảng trống sâu, phạt góc lệch tâm */
        float dev_penalty = ABS_F((float)center_deg - 90.0f) * 2.0f;
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
 

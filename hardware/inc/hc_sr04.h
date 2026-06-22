
#ifndef MYLIB_INC_HC_SR04_H_
#define MYLIB_INC_HC_SR04_H_
#include "main.h"
#include "types.h"
#include "encoder.h"
typedef struct
{
    _vo u32 val1;
    _vo u32 val2;
    _vo u32 diff;
    _vo u32 d;
    _vo u8 first_cap;
     u8 done;
     u8 busy;
     u32 start_ms;

}HC_t;

extern HC_t hc[2];
extern TIM_HandleTypeDef htim1; 
void hcsr04_read();
void hcsr04_init();
void hcsr04_check_timeout(void);

/* Ngưỡng (mm) coi là "hố" — sàn thật thường ~ vài cm tới vài chục cm
 * dưới đáy xe; chỉnh theo chiều cao gắn cảm biến thực tế. */
#define CLIFF_MM        80u

/* Trả 1 nếu BẤT KỲ cảm biến đáy nào báo hố (đọc > CLIFF_MM, kể cả
 * 9999 = mất echo). Dùng dữ liệu mới nhất trong hc[0].d/hc[1].d —
 * cần hcsr04_read() được bơm định kỳ (~vài chục ms) ở vòng lặp chính. */
static inline u8 hcsr04_cliff_detected(void)
{
    return (hc[0].d > CLIFF_MM) || (hc[1].d > CLIFF_MM);
}

#endif /* MYLIB_INC_HC_SR04_H_ */

#include "../inc/OLED.h"

#include "math.h"
#include "stdio.h"

#define PI 3.14159265f

// Khai bao font chu mac dinh thuong di kem voi thu vien ssd1306 (ssd1306_fonts.h)
// Neu project cua ban dung font khac thi doi ten Font_7x10 thanh font ban co (VD: Font_6x8)
extern SSD1306_Font_t Font_7x10; 

void OLED_Init(void) {
    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_UpdateScreen();
}

void OLED_DrawRadarMap(void) {
    // 1. Xoa toan bo man hinh
    ssd1306_Fill(Black);

    // 2. Ve cac duong luoi Radar (Gia lap luoi quet cho chuyen nghiep)
    // Ve cham tam xe
    ssd1306_DrawCircle(OLED_ORIGIN_X, OLED_ORIGIN_Y, 2, White);
    
    // Ve cac tia dinh huong: 45 do, 90 do, 135 do de de nhin
    ssd1306_Line(OLED_ORIGIN_X, OLED_ORIGIN_Y, OLED_ORIGIN_X - 42, OLED_ORIGIN_Y - 42, White); // Tia 135 do (Trai)
    ssd1306_Line(OLED_ORIGIN_X, OLED_ORIGIN_Y, OLED_ORIGIN_X, 3, White);                       // Tia 90 do (Thang)
    ssd1306_Line(OLED_ORIGIN_X, OLED_ORIGIN_Y, OLED_ORIGIN_X + 42, OLED_ORIGIN_Y - 42, White); // Tia 45 do (Phai)

    // Bien luu toa do diem truoc do de ve duong Line noi (Contour)
    int prev_x = -1;
    int prev_y = -1;

    // 3. Quet mang Lidar_Map va ve do thi
    for (int angle = 0; angle <= 180; angle += 10) {
        uint16_t dist = Lidar_Map[angle];

        // Xu ly nhieu: Neu Lidar loi (=0) hoac qua xa, ep ve max tam nhin radar
        if (dist == 0 || dist > MAX_RADAR_DIST_MM) {
            dist = (uint16_t)MAX_RADAR_DIST_MM; 
        }

        // Tinh chieu dai ban kinh Pixel tren man hinh
        float r_pixel = ((float)dist / MAX_RADAR_DIST_MM) * RADAR_RADIUS_PIXEL;
        
        // Doi do (Degree) sang Radian
        float rad = (float)angle * PI / 180.0f;

        // Tinh toa do x, y
        // cos(0) = 1 -> phia ben phai
        // sin dung de xac dinh do cao Y. Man hinh y di xuong nen phai lay Y_Goc TRU di
        int x = OLED_ORIGIN_X + (int)(r_pixel * cos(rad));
        int y = OLED_ORIGIN_Y - (int)(r_pixel * sin(rad));

        // Gioi han toa do de khong bi tran RAM cua OLED
        if (x < 0) x = 0;
        if (x > 127) x = 127;
        if (y < 0) y = 0;
        if (y > 63) y = 63;

        // Noi 2 diem lien tiep lai voi nhau theo yeu cau cua ban
        if (prev_x != -1 && prev_y != -1) {
            ssd1306_Line(prev_x, prev_y, x, y, White);
        }

        // To dam diem chot hien tai
        ssd1306_DrawPixel(x, y, White);
        // Ban cung co the dung DrawCircle nho de lam noi bat cac diem quet
        // ssd1306_DrawCircle(x, y, 1, White);

        // Cap nhat diem cu
        prev_x = x;
        prev_y = y;
    }

    // 4. In thong so len goc man hinh (Goc tren ben trai)
    // In ra khoang cach cua tia 90 do (tia chih dien)
    char textBuff[20];
    sprintf(textBuff, "%dmm", Lidar_Map[90]);
    ssd1306_SetCursor(0, 0); 
    
    // Luu y: Neu ban khong dung Font_7x10, hay sua thanh Font_6x8 cua thu vien
    ssd1306_WriteString(textBuff, Font_7x10, White);

    // 5. Day du lieu tu Buffer len man hinh hien thi
    ssd1306_UpdateScreen();
}
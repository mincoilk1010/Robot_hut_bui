/*
 * nav.c
 *
 *  Created on: Jun 17, 2026
 *      Author: GB Center
 */

/*
#include "nav.h"
#include "control.h"

#include "vl53_scan.h"
#include "hc_sr04.h"
#include "map.h"
#include <math.h>

Nav_t nav={N_BOOT,1,0,0,0,0,0,0};

static void _nenter(NavSt_t s){nav.st=s;nav.t0=HAL_GetTick();}

static float _dist(void)
{
	float dx=pose.x-nav.x0,dy=pose.y-nav.y0;
	return sqrtf(dx*dx+dy*dy);
}

static float _hh_itg=0;

static void _fwd_ctrl(void)
{
    float ye=angle_diff(nav.lane_yaw,yaw);
    _hh_itg=limit(_hh_itg+ye*dt_s,-20.0f,20.0f);
    g_sp_v=NAV_SPEED;
    g_sp_w=limit(0.030f*ye+0.002f*_hh_itg,-0.40f,0.40f);
}

void nav_init(void){
    nav.st=N_BOOT;
    nav.dir=1;
    nav.row=0;
    nav.done=0;
    nav.t0=HAL_GetTick();
    nav.lane_yaw=0;
    _hh_itg=0;
}

void nav_task(void)
{
    uint32_t now=HAL_GetTick();


    if(turn_final_yaw()&&(nav.st==N_FWD||nav.st==N_CROSS)){
        g_sp_v=0;g_sp_w=0;_hh_itg=0;_nenter(N_CLIFF);return;
    }

    switch(nav.st){


    case N_BOOT:
        g_sp_v=0;g_sp_w=0;
        if(now-nav.t0>800u){
            nav.lane_yaw=yaw;nav.x0=pose.x;nav.y0=pose.y;
            _nenter(N_FWD);
        }
        break;

    case N_FWD:{
        _fwd_ctrl();
        u16 front=scanner_front();
        float dist=_dist();
        if(dist>2.8f||nav.row>=NAV_MAX_ROWS||(front>0u&&front<NAV_OBS_MM)){
            g_sp_v=0;g_sp_w=0;_hh_itg=0;_nenter(N_BRAKE);
        }
        break;
    }


    case N_BRAKE:
        g_sp_v=0;g_sp_w=0;
        if(now-nav.t0>250u){

            HAL_Delay(30);
            float td=(nav.dir==1)?-90.0f:90.0f;
            turn_start(td);
            _nenter(N_TURN90);
        }
        break;
    case N_TURN90:
        turn_task();
        if(turn_done()){
            nav.lane_yaw=turn_final_yaw();
            nav.x0=pose.x;nav.y0=pose.y;
            g_sp_w=0;_hh_itg=0;
            _nenter(N_CROSS);
        }
        break;


    case N_CROSS:{
        float d=_dist();
        if(d<NAV_ROW_M&&now-nav.t0<6000u){
            float remain=NAV_ROW_M-d;
            float v=(remain<0.08f)?NAV_SPEED*(remain/0.08f+0.2f):NAV_SPEED;
            float ye=angle_diff(nav.lane_yaw,yaw);
            _hh_itg=limit(_hh_itg+ye*dt_s,-15.0f,15.0f);
            g_sp_v=limit(v,0.04f,NAV_SPEED);
            g_sp_w=limit(0.030f*ye+0.002f*_hh_itg,-0.35f,0.35f);
        }else{
            g_sp_v=0;g_sp_w=0;_hh_itg=0;
            nav.row++;
            if(nav.row>=NAV_MAX_ROWS){_nenter(N_DONE);break;}
            HAL_Delay(30);
            float td=(nav.dir==1)?-90.0f:90.0f;
            turn_start(td);
            nav.dir=-nav.dir;
            _nenter(N_TURN90B);
        }
        break;
    }


    case N_TURN90B:
        turn_task();
        if(turn_done()){
            nav.lane_yaw=turn_final_yaw();
            nav.x0=pose.x;nav.y0=pose.y;
            g_sp_w=0;_hh_itg=0;
            _nenter(N_FWD);
        }
        break;


    case N_CLIFF:
        g_sp_v=-NAV_SPEED*0.5f;g_sp_w=0;
        if(now-nav.t0>600u){
            g_sp_v=0;g_sp_w=0;HAL_Delay(30);
            turn_start(90.0f);_nenter(N_TURN90);
        }
        break;


    case N_DONE:
        g_sp_v=0;g_sp_w=0;nav.done=1;
        PH_activate();


        break;
    }
}
*/

#include "nav.h"
#include "control.h"

#include "vl53_scan.h"
#include "hc_sr04.h"
#include "map.h"
#include "avoid.h"
#include <math.h>

Nav_t nav={.st=N_BOOT,.dir=1,.row=0,.lane_yaw=0,.x0=0,.y0=0,.t0=0,.done=0};

static void _nenter(NavSt_t s){nav.st=s;nav.t0=HAL_GetTick();}

static float _dist(void)
{
	float dx=pose.x-nav.x0,dy=pose.y-nav.y0;
	return sqrtf(dx*dx+dy*dy);
}

static float _hh_itg=0;

static void _fwd_ctrl(void)
{
    float ye=angle_diff(nav.lane_yaw,yaw);
    _hh_itg=limit(_hh_itg+ye*dt_s,-20.0f,20.0f);
    g_sp_v=NAV_SPEED;
    g_sp_w=limit(0.030f*ye+0.002f*_hh_itg,-0.40f,0.40f);
}

/* ── Chọn hướng quay né vật cản: bên nào THOÁNG hơn ──
 * sc.data[] đã chứa kết quả wide-scan (góc 0..180°, bước SC_STEP).
 * Lấy avg(mm) bên trái (>90°, hướng quay +90° = trái) và bên phải
 * (<90°, hướng quay -90° = phải), trừ no-echo (9999) ra khỏi trung
 * bình để không làm méo kết quả vì "không có gì" không có nghĩa là
 * "rất thoáng" một cách đáng tin. Né về bên có avg lớn hơn.
 * Trả +1 → quay trái (+90°), -1 → quay phải (-90°). */
static i8 _pick_avoid_dir(void)
{
    long sum_l=0; int n_l=0;
    long sum_r=0; int n_r=0;
    int n=(SC_W_MAX-SC_W_MIN)/SC_STEP+1;

    for(int i=0;i<n;i++){
        u8 deg=(u8)(SC_W_MIN+i*SC_STEP);
        u16 v=sc.data[i];
        if(v==9999u) continue;          /* bỏ no-echo khỏi trung bình */
        if(deg>90u){ sum_l+=v; n_l++; }
        else if(deg<90u){ sum_r+=v; n_r++; }
    }

    /* Ưu tiên dùng Avoid_FindBestGap() nếu tìm được khe hở hợp lệ:
     * né theo dấu redirect_deg (lệch so với 90° = hướng thẳng). */
    Gap_t gap;
    if(Avoid_FindBestGap(&gap)){
        return (gap.redirect_deg >= 0.0f) ? 1 : -1;
    }

    /* Fallback: không có khe đủ rộng → vẫn chọn bên có avg lớn hơn
     * (đỡ tệ hơn) để né, ưu tiên có dữ liệu hơn không. */
    float avg_l=(n_l>0)?(float)sum_l/n_l:0.0f;
    float avg_r=(n_r>0)?(float)sum_r/n_r:0.0f;
    if(n_l==0 && n_r==0) return nav.dir;     /* không có gì để so, giữ hướng cũ */
    return (avg_l>=avg_r) ? 1 : -1;
}

void nav_init(void){
    nav.st=N_BOOT;
    nav.dir=1;
    nav.row=0;
    nav.done=0;
    nav.t0=HAL_GetTick();
    nav.lane_yaw=0;
    _hh_itg=0;
}

void nav_task(void)
{
    uint32_t now=HAL_GetTick();

    /* ── Cliff: ưu tiên ── đọc THẬT từ HC-SR04 đáy xe, không dùng
     * turn_final_yaw() (đó là góc yaw, không phải cờ phát hiện hố). */
    if(hcsr04_cliff_detected()&&(nav.st==N_FWD||nav.st==N_CROSS)){
        g_sp_v=0;g_sp_w=0;_hh_itg=0;_nenter(N_CLIFF);return;
    }

    switch(nav.st){

    /* ── Boot: chờ 800ms cho gyro ổn định ── */
    case N_BOOT:
        g_sp_v=0;g_sp_w=0;
        if(now-nav.t0>800u){
            nav.lane_yaw=yaw;nav.x0=pose.x;nav.y0=pose.y;
            _nenter(N_FWD);
        }
        break;

    /* ── Tiến thẳng + servo scan song song ── */
    case N_FWD:{
        _fwd_ctrl();
        u16 front=scanner_front();
        float dist=_dist();
        if(dist>2.8f||nav.row>=NAV_MAX_ROWS||(front>0u&&front<NAV_OBS_MM)){
            g_sp_v=0;g_sp_w=0;_hh_itg=0;_nenter(N_BRAKE);
        }
        break;
    }

    /* ── Phanh 250ms, rồi CHỌN hướng né theo bên thoáng hơn ── */
    case N_BRAKE:
        g_sp_v=0;g_sp_w=0;
        if(now-nav.t0>250u){
            /* Chờ 30ms cho gyro settle */
            HAL_Delay(30);
            nav.dir=_pick_avoid_dir();          /* ★ chọn động, không cố định */
            float td=(nav.dir==1)?90.0f:-90.0f; /* +1=trái(+90°), -1=phải(-90°) */
            turn_start(td);
            _nenter(N_TURN90);
        }
        break;

    /* ── Quay 90° chính xác ── */
    case N_TURN90:
        turn_task();
        if(turn_done()){
            nav.lane_yaw=turn_final_yaw();  /* hướng khi đi ngang */
            nav.x0=pose.x;nav.y0=pose.y;
            g_sp_w=0;_hh_itg=0;
            _nenter(N_CROSS);
        }
        break;

    /* ── Tiến ngang 1 row ── */
    case N_CROSS:{
        float d=_dist();
        if(d<NAV_ROW_M&&now-nav.t0<6000u){
            float remain=NAV_ROW_M-d;
            float v=(remain<0.08f)?NAV_SPEED*(remain/0.08f+0.2f):NAV_SPEED;
            float ye=angle_diff(nav.lane_yaw,yaw);
            _hh_itg=limit(_hh_itg+ye*dt_s,-15.0f,15.0f);
            g_sp_v=limit(v,0.04f,NAV_SPEED);
            g_sp_w=limit(0.030f*ye+0.002f*_hh_itg,-0.35f,0.35f);
        }else{
            g_sp_v=0;g_sp_w=0;_hh_itg=0;
            nav.row++;
            if(nav.row>=NAV_MAX_ROWS){_nenter(N_DONE);break;}
            HAL_Delay(30);
            float td=(nav.dir==1)?-90.0f:90.0f;
            turn_start(td);
            nav.dir=-nav.dir;
            _nenter(N_TURN90B);
        }
        break;
    }

    /* ── Quay về hướng hàng mới ── */
    case N_TURN90B:
        turn_task();
        if(turn_done()){
            nav.lane_yaw=turn_final_yaw();
            nav.x0=pose.x;nav.y0=pose.y;
            g_sp_w=0;_hh_itg=0;
            _nenter(N_FWD);
        }
        break;

    /* ── Cliff: lùi ngắn rồi quay ── */
    case N_CLIFF:
        g_sp_v=-NAV_SPEED*0.5f;g_sp_w=0;
        if(now-nav.t0>600u){
            g_sp_v=0;g_sp_w=0;HAL_Delay(30);
            turn_start(90.0f);_nenter(N_TURN90);
        }
        break;

    /* ── Done: dừng + giữ vị trí ── */
    case N_DONE:
        g_sp_v=0;g_sp_w=0;nav.done=1;
        PH_activate();


        break;
    }
}

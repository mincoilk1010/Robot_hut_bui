/*
 * nav.c
 *
 *  Created on: Jun 17, 2026
 *      Author: GB Center
 */


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

    /* ── Cliff: ưu tiên ── */
    if(turn_final_yaw()&&(nav.st==N_FWD||nav.st==N_CROSS)){
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

    /* ── Phanh 250ms ── */
    case N_BRAKE:
        g_sp_v=0;g_sp_w=0;
        if(now-nav.t0>250u){
            /* Chờ 30ms cho gyro settle */
            HAL_Delay(30);
            float td=(nav.dir==1)?-90.0f:90.0f;
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

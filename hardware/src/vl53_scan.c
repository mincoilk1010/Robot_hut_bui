/*
 * vl53_scan.c
 *
 *  Created on: Jun 17, 2026
 *      Author: GB Center
 */


#include "vl53_scan.h"
#include "map.h"
#include "control.h"       
#include <math.h>
 
Scanner_t sc={0};
 
static void _svo(u8 deg)
{

    if(deg>180u)deg=180u;
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,500u+(u32)deg*2000u/180u);
}
static void _enter_narrow(void)
{
    sc.mode=SC_NARROW;
    sc.amin=SC_N_MIN;
    sc.amax=SC_N_MAX;
    sc.angle=SC_N_MIN;
    sc.state=SC_MOVE;}
static void _enter_wide(void)
{
    sc.mode=SC_WIDE;
    sc.amin=SC_W_MIN;
    sc.amax=SC_W_MAX;
    sc.angle=SC_W_MIN;
    sc.clear_cnt=0;
    sc.state=SC_MOVE;}
 
void scanner_init(void)
{
    for(int i=0;i<37;i++) sc.data[i]=9999u;
    _enter_narrow();
    sc.done=0; 
    sc.cycle=0;
    _svo(90u); 
    HAL_Delay(50);
}
 
void scanner_task(void)
{
    uint32_t now=HAL_GetTick();
    switch(sc.state){
    case SC_IDLE: 
        sc.angle=sc.amin; 
        sc.state=SC_MOVE; 
        break;
    case SC_MOVE: 
        _svo(sc.angle); 
        sc.t_servo=now; 
        sc.state=SC_WAIT; 
        break;
    case SC_WAIT:
        if(now-sc.t_servo>=SC_WAIT_MS) sc.state=SC_READ;
         break;
    case SC_READ:{
        uint16_t d=(uint16_t)readRangeSingleMillimeters(NULL);
        if(d==0||d>2000u) d=9999u;
        u8 idx=sc.angle/SC_STEP;
        if(idx<37u) sc.data[idx]=d;
        map_update(pose.x,pose.y,pose.theta,(float)sc.angle,d);
 
        /* Chuyển mode */
        if(sc.mode==SC_NARROW && d<SC_OBS_MM){
            _enter_wide();
             return;
        }
        if(sc.mode==SC_WIDE && sc.angle>=SC_N_MIN && sc.angle<=SC_N_MAX){
            if(d>SC_CLEAR_MM) sc.clear_cnt++;
            else sc.clear_cnt=0;
            if(sc.clear_cnt>=5u)
            {
                _enter_narrow();
                return;}
        }
        /* Góc tiếp theo */
        if(sc.angle<sc.amax){ sc.angle+=SC_STEP; sc.state=SC_MOVE; }
        else{
            _svo(90u); sc.done=1; sc.cycle++;
            map_mark_robot(pose.x,pose.y);
            sc.angle=sc.amin; 
            sc.state=SC_MOVE; 
            sc.done=0;
        }
        break;
    }}
}
 
u16  scanner_get(u8 deg){
    u8 i=deg/SC_STEP;
    return i<37u?sc.data[i]:9999u;
}
u16  scanner_front(void){
    return scanner_get(90u);
}
u8   scanner_has_obs(void)
{
    for(u8 a=SC_N_MIN;a<=SC_N_MAX;a+=SC_STEP)
        if(scanner_get(a)<SC_OBS_MM) return 1;
    return 0;
}
ScMode_t scanner_mode(void){return sc.mode;}

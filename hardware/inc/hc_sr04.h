/*
 * hc_sr04.h
 *
 *  Created on: May 14, 2026
 *      Author: GB Center
 */

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
    _vo u8 done;
    _vo u8 f;

}HC_t;

extern HC_t hc[2];
extern TIM_HandleTypeDef htim1; 
void hcsr04_read();
void hcsr04_init();


#endif /* MYLIB_INC_HC_SR04_H_ */

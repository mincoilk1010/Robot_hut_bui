/*
 * delay.h
 *
 *  Created on: Jun 8, 2026
 *      Author: GB Center
 */

#ifndef INC_DELAY_H_
#define INC_DELAY_H_
#include "types.h"
void delay_init();
void delay_ms(u32 mssec);
void delay_us(u32 us);
u32 millis(void);
u32 micros(void);

#endif /* INC_DELAY_H_ */

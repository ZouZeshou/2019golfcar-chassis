#ifndef __DELAY_H_
#define __DELAY_H_
#include "stm32f4xx_hal.h"


void xdelay_ms(unsigned int t);
void xdelay_us(uint32_t nus);
void delay_ms(uint16_t nms);
void delay_us(uint32_t nus);

#endif



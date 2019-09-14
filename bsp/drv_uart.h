#ifndef DRV_UART__H
#define DRV_UART__H

#include "stm32f4xx.h"

void USART1_Enable(void);
void USART3_Enable(void);
void USART1_Send_Char(uint8_t u8_char);
#endif

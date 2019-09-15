#ifndef DRV_UART__H
#define DRV_UART__H

#include "stm32f4xx.h"

void USART_Enable(UART_HandleTypeDef *huart,uint8_t * buffer_addr);
void USART_Send_Char(UART_HandleTypeDef *huart,uint8_t u8_char);
void USART_DMA_Enable(UART_HandleTypeDef *huart,DMA_HandleTypeDef *hdma,uint8_t * buffer_addr,uint8_t data_size);
void USART_IDLE_IRQ(UART_HandleTypeDef *huart,uint8_t * buffer_addr,uint8_t data_size);
extern uint8_t Usart1buff[100];
extern uint8_t Usart2buff[100];
extern uint8_t Usart3buff[100];
extern uint8_t Usart6buff[100];
#endif

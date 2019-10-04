#ifndef __PROTOCOL_H
#define __PROTOCOL_H
#include "stm32f4xx.h"
union w4data
{
	float f;
	uint8_t u[4];
};
struct data_to_send
{
	union w4data pos_x;
	union w4data pos_y;
	union w4data angle;
	
	uint8_t finish_run;//0-notfinish  1-fininsh
	uint8_t ball_color;//1-black 2-white 3-pink 4-environment
	uint8_t colorsensor_ready;//0-notready  1-ready

};
struct data_receive
{
	uint8_t start_run;//1-run from right 2-run from left
	uint8_t ready_to_shoot;//0-not ready 1-ready 
	uint8_t ready_to_shoot_last;
	uint8_t black_or_white;//0-black 1-white
};
void deal_receive_data(uint8_t *buffer);
void send_data_to_gimbal(UART_HandleTypeDef *huart);
extern struct data_to_send s_send_data;
extern struct data_receive s_receive_data;






#endif

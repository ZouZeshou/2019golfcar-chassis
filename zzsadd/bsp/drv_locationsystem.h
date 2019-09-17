#ifndef __DRV_LOCATION_H
#define __DRV_LOCATION_H
#include "stm32f4xx.h"

struct posture_data
{
	float pos_x;
	float pos_y;
	float zangle;
	float xangle;
	float yangle;
	float w_z;
};


extern struct posture_data s_posture;


void get_loca_sys_data(uint8_t * buffer);
#endif

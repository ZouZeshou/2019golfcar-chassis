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
	
	float ang_last;
	float ang_tol;
	int cir_num;
};


extern struct posture_data s_posture;


void get_loca_sys_data(uint8_t * buffer);
void angle_to_continue(struct posture_data *s_pos);
#endif

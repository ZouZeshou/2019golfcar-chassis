#ifndef __DRV_COLOR_H
#define __DRV_COLOR_H
#include "stm32f4xx.h"
struct s_colour_sensor_data
{
	int Start;
	int Lux;
	int CT;
	int color;
	int END;
};
extern struct s_colour_sensor_data s_color_data;
void colour_sensor_init(void);
void deal_coloursensor_data(uint8_t * buffer);

#endif

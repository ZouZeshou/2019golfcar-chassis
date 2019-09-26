#ifndef __DRV_COLOR_H
#define __DRV_COLOR_H
#include "stm32f4xx.h"
//1-black 2-white 3-pink 4-environment
#define BLACK 1
#define WHITE 2
#define PINK 3
#define ENVIRONMENT 4
struct s_colour_sensor_data
{
	int Start;
	int Lux;
	int CT;
	int color;
	int END;
	int ball_color;
};
enum color{pink,black,white,enviroment};
extern struct s_colour_sensor_data s_color_data;
void colour_sensor_init(void);
void deal_coloursensor_data(uint8_t * buffer);
int detect_the_color(struct s_colour_sensor_data *s_color);

#endif

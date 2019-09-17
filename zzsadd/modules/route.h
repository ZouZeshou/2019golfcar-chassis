#ifndef __ROUTE_H
#define __ROUTE_H
#include "stm32f4xx.h"
#include "chassis.h"
struct route_point
{
	int x[50];
	int y[50];
};

void route_init(void);
void design_point_of_route(struct route_point *s_route,int direction,int point_num,
	int radius_1,int radius_2,int radius_3);
void update_point(struct route_point *s_route,int *point_addr,int pos_x,int pos_y,
	float accuracy,int jam_time,int point_num);
void calculate_motor_current(struct pid *s_left_pid,struct pid *s_right_pid,struct pid *s_ang_pid,
	int aim_point_x,int aim_point_y,int pos_x,int pos_y,float pos_angle,int speed,
		struct s_motor_data *s_left,struct s_motor_data *s_right);
extern struct route_point s_route;

#endif

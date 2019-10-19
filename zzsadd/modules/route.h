#ifndef __ROUTE_H
#define __ROUTE_H
#include "stm32f4xx.h"
#include "chassis.h"
#include "ramp.h"
struct route_point
{
	int x[100];
	int y[100];
	float angle[100];
};
struct point_coordinate
{
	int x;
	int y;
};

void route_init(void);
void design_point_of_route(struct route_point *s_route,int direction,int point_num,
	int radius_1,int radius_2,int radius_3);
void design_point_of_helix_route(struct route_point *s_route,int direction,int point_num,
	int alpha,int beta,float circle_num);
void update_point(struct route_point *s_route,int *point_addr,int pos_x,int pos_y,
	float accuracy,int jam_time,int point_num);
void calculate_motor_current(struct pid *s_left_pid,struct pid *s_right_pid,struct pid *s_ang_pid,
	int aim_point_x,int aim_point_y,float aim_point_angle,int pos_x,int pos_y,float pos_angle,int speed,
		int jam_back_time,struct s_motor_data *s_left,struct s_motor_data *s_right);
struct point_coordinate choose_destination(uint8_t left_right,uint8_t black_white,uint8_t bucket_num,int radius);
float choose_detination_by_circle(uint8_t left_right,uint8_t black_white,uint8_t bucket_num);	
	
extern struct route_point s_route;
extern struct pid s_angle_pid;
extern struct point_coordinate s_destination;
extern ramp_t s_left_ramp;
extern ramp_t s_right_ramp;
#endif

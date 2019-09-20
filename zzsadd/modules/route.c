#include "route.h"
#include "math.h"
#include "pid.h"
#include "STMGood.h"
#include "drv_uart.h"
#include "usr_task.h"
#define PI 3.1415926
//以出发点为（0,0）建立坐标系 x(-2400,2400) y(0,4800)
struct route_point s_route={0};
struct pid s_angle_pid;
int angle_pid_debug = 0;
int motor_pid_debug = 0;
int jam_back = 0;
/**
 * @brief route_init
 * @param 
 * @return None
 * @attention None
 */
void route_init(void)
{
	pid_struct_init(&s_angle_pid,3000,200,70,0,0);
}
/**
 * @brief design every point of route 以出发点为原点计算路径上各点的x,y坐标
 * @param direction 向左为1向右为0 point_num 点的个数 radius_1 第一圈半径 radius_2 第二圈半径
 * @return None
 * @attention None
 */
void design_point_of_route(struct route_point *s_route,int direction,int point_num,
	int radius_1,int radius_2,int radius_3)
{
	if(direction==1)
	{
		for(int i=0;i<= (point_num/3-1);i++)
		{
			s_route->x[i] = radius_1 * cos(PI-2*PI/(point_num/3)*(i+1-point_num/12));
			s_route->y[i] = radius_1 * sin(PI-2*PI/(point_num/3)*(i+1-point_num/12))+2200;
		}
		for(int i=point_num/3;i<= (2*point_num/3-1);i++)
		{
			s_route->x[i] = radius_2 * cos(PI-2*PI/(point_num/3)*(i+1-point_num/12));
			s_route->y[i] = radius_2 * sin(PI-2*PI/(point_num/3)*(i+1-point_num/12))+2200;
		}
		for(int i=2*point_num/3;i<= (point_num-1);i++)
		{
			s_route->x[i] = radius_3 * cos(PI-2*PI/(point_num/3)*(i+1-point_num/12));
			s_route->y[i] = radius_3 * sin(PI-2*PI/(point_num/3)*(i+1-point_num/12))+2200;
		}
	}
	else
	{
		for(int i=0;i<= (point_num/3-1);i++)
		{
			s_route->x[i] = radius_1 * cos(2*PI/(point_num/3)*(i+1-point_num/12));
			s_route->y[i] = radius_1 * sin(2*PI/(point_num/3)*(i+1-point_num/12))+2200;
		}
		for(int i=point_num/3;i<= (2*point_num/3-1);i++)
		{
			s_route->x[i] = radius_2 * cos(2*PI/(point_num/3)*(i+1-point_num/12));
			s_route->y[i] = radius_2 * sin(2*PI/(point_num/3)*(i+1-point_num/12))+2200;
		}
		for(int i=2*point_num/3;i<= (point_num-1);i++)
		{
			s_route->x[i] = radius_3 * cos(2*PI/(point_num/3)*(i+1-point_num/12));
			s_route->y[i] = radius_3 * sin(2*PI/(point_num/3)*(i+1-point_num/12))+2200;
		}
	}
}
/**
 * @brief updata the point 
 * @param 
 * @return None
 * @attention None
 */
void update_point(struct route_point *s_route,int *point_addr,int pos_x,int pos_y,
	float accuracy,int jam_time,int point_num)
{
	static int jam_counter;
	static float distance;
	static float jam_distance_now,jam_distance_last;
	static int nearby_point_num;
	distance = sqrt((s_route->x[*point_addr]-pos_x)*(s_route->x[*point_addr]-pos_x)
							+(s_route->y[*point_addr]-pos_y)*(s_route->y[*point_addr]-pos_y));
	if(distance <= accuracy)
	{
		*point_addr = *point_addr + 1;
		jam_counter = 0;
	}
	else
	{
		if(jam_counter++ >= jam_time)
		{
			jam_back = 1;
			for(int i=*point_addr-3;i<=*point_addr+3;i++)
			{
				jam_distance_now = sqrt((s_route->x[i]-pos_x)*(s_route->x[i]-pos_x)
								+(s_route->y[i]-pos_y)*(s_route->y[i]-pos_y));
				if(jam_distance_now < jam_distance_last)
				{
					nearby_point_num = i;
				}
				jam_distance_last = jam_distance_now;
			}
			*point_addr = nearby_point_num;
			jam_counter = 0;
		}
	}
	if(*point_addr >= point_num)
	{
		*point_addr = point_num;
	}
//	printf("aim x %d y %d\r\n",s_route->x[*point_addr],s_route->y[*point_addr]);
//	printf("now x %d y %d\r\n",pos_x,pos_y);
//	printf("pointnum %d\r\n",*point_addr);
}
/**
 * @brief calculate_motor_current 
 * @param 
 * @return None
 * @attention None
 */
void calculate_motor_current(struct pid *s_left_pid,struct pid *s_right_pid,struct pid *s_ang_pid,
	int aim_point_x,int aim_point_y,int pos_x,int pos_y,float pos_angle,int speed,int jam_back_time,
		struct s_motor_data *s_left,struct s_motor_data *s_right)
{
	static int jam_back_counter=0;
	static int cir_num,init=0;
	static float aim_angle,aim_ang_last,aim_ang_tol,angle_out;
	aim_ang_last = aim_angle;  
	aim_angle = -atan2f((aim_point_x-pos_x),(aim_point_y-pos_y))*180/PI;
	if(init)
	{
		if(aim_angle - aim_ang_last > 300)
		{
			cir_num--;
		}
		else if(aim_angle - aim_ang_last < -300)
		{
			cir_num++;
		}
		aim_ang_tol = aim_angle + cir_num * 360;
	}
	else
	{
		init = 1;
	}
	if(angle_pid_debug==1)
	{
		pid_struct_init(s_ang_pid,V1,200,P,I,D);
	}
	angle_out = pid_calculate(s_ang_pid,pos_angle,aim_ang_tol);
//	printf("aimangle %.2f\r\n",aim_angle);
//	printf("totalang %.2f\r\n",pos_angle);
//	printf("ang_out %.2f\r\n",angle_out);
	if(motor_pid_debug==1)
	{
		pid_struct_init(s_left_pid,V1,200,P,I,D);
		pid_struct_init(s_right_pid,V1,200,P,I,D);
	}
	if(jam_back==1)
	{
		if(jam_back_counter++ >= jam_back_time)
		{
			jam_back = 0;
			jam_back_counter = 0;
		}
		s_left->target_speed = - speed;
		s_right->target_speed = speed;
	}
	else
	{
		s_left->target_speed = speed - angle_out;
		s_right->target_speed = -speed - angle_out;
	}
	
	s_left->out_current = (int)(pid_calculate(s_left_pid,s_left->back_speed,s_left->target_speed));
	s_right->out_current = (int)(pid_calculate(s_right_pid,s_right->back_speed,s_right->target_speed));

	
}
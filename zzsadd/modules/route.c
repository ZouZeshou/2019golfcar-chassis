#include "route.h"
#include "math.h"
#include "pid.h"
#include "STMGood.h"
#include "drv_uart.h"
#include "usr_task.h"
#include "drv_locationsystem.h"
#include "drv_coloursensor.h"
#include "protocol.h"
#define PI 3.1415926
//以出发点为（0,0）建立坐标系 x(-2400,2400) y(0,4800)
struct route_point s_route={0};
struct point_coordinate s_destination = {0};
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
	pid_struct_init(&s_angle_pid,8000,200,60,0,0);
}
/**
 * @brief design the destination
 * @param left_right从左、右启动 black_white蓝-BLACK，红-WHITE bucket_num桶的编号 radius目标点半径
 * @return 目标点的坐标
 * @attention None
 */
float choose_detination_by_circle(uint8_t left_right,uint8_t black_white,uint8_t bucket_num)
{
	switch(bucket_num)
	{
		case 12:
		{
			if(black_white==BLACK)
			{
				return 2.0;
			}
			else if(black_white==WHITE)
			{
				return 2.5;
			}
			break;
		}
		case 23:
		{
			if((left_right==RIGHT&&black_white==WHITE)||(left_right==LEFT&&black_white==BLACK))
			{
				return 2.25;
			}
			else if((left_right==RIGHT&&black_white==BLACK)||(left_right==LEFT&&black_white==WHITE))
			{
				return 2.75;
			}
			break;
		}
		case 34:
		{
			if(black_white==BLACK)
			{
				return 2.5;
			}
			else if(black_white==WHITE)
			{
				return 2.0;
			}
			break;
		}
		case 14:
		{
			if((left_right==RIGHT&&black_white==WHITE)||(left_right==LEFT&&black_white==BLACK))
			{
				return 2.75;
			}
			else if((left_right==RIGHT&&black_white==BLACK)||(left_right==LEFT&&black_white==WHITE))
			{
				return 2.25;
			}
			break;
		}
		case 13:
		{
			if((left_right==RIGHT&&black_white==WHITE)||(left_right==LEFT&&black_white==BLACK))
			{
				return 2.38;
			}
			else if((left_right==RIGHT&&black_white==BLACK)||(left_right==LEFT&&black_white==WHITE))
			{
				return 2.125;
			}
			break;
		}
		case 24:
		{
			if((left_right==RIGHT&&black_white==WHITE)||(left_right==LEFT&&black_white==BLACK))
			{
				return 2.125;
			}
			else if((left_right==RIGHT&&black_white==BLACK)||(left_right==LEFT&&black_white==WHITE))
			{
				return 2.38;
			}
			break;
		}
		default:
			break;
		
	}
	return 0;
}
/**
 * @brief design the destination
 * @param left_right从左、右启动 black_white蓝-BLACK，红-WHITE bucket_num桶的编号 radius目标点半径
 * @return 目标点的坐标
 * @attention None
 */
struct point_coordinate choose_destination(uint8_t left_right,uint8_t black_white,uint8_t bucket_num,int radius)
{
	static struct point_coordinate s_desti={0};
	
	switch(bucket_num)
	{
		case 12:
		{
			switch(black_white)
			{
				case BLACK:
				{
					s_desti.x = radius * cos(-90*PI/180);
					s_desti.y = radius * sin(-90*PI/180) + 2200;
					break;
				}
				case WHITE:
				{
					s_desti.x = radius * cos(90*PI/180);
					s_desti.y = radius * sin(90*PI/180) + 2200;
					break;
				}
				default:
					break;
			}
			break;
		}
		case 23:
		{
			switch(black_white)
			{
				case BLACK:
				{
					s_desti.x = radius * cos(180*PI/180);
					s_desti.y = radius * sin(180*PI/180) + 2200;
					break;
				}
				case WHITE:
				{
					s_desti.x = radius * cos(0*PI/180);
					s_desti.y = radius * sin(0*PI/180) + 2200;
					break;
				}
				default:
					break;
			}
			break;
		}
		case 34:
		{
			switch(black_white)
			{
				case BLACK:
				{
					s_desti.x = radius * cos(90*PI/180);
					s_desti.y = radius * sin(90*PI/180) + 2200;
					break;
				}
				case WHITE:
				{
					s_desti.x = radius * cos(-90*PI/180);
					s_desti.y = radius * sin(-90*PI/180) + 2200;
					break;
				}
				default:
					break;
			}
			break;
		}
		case 14:
		{
			switch(black_white)
			{
				case BLACK:
				{
					s_desti.x = radius * cos(0*PI/180);
					s_desti.y = radius * sin(0*PI/180) + 2200;
					break;
				}
				case WHITE:
				{
					s_desti.x = radius * cos(180*PI/180);
					s_desti.y = radius * sin(180*PI/180) + 2200;
					break;
				}
				default:
					break;
			}
			break;
		}
		case 13:
		{
				if(left_right==RIGHT)
				{
					s_desti.x = radius * cos(45*PI/180);
					s_desti.y = radius * sin(45*PI/180) + 2200;
				}
				else if(left_right==LEFT)
				{
					s_desti.x = radius * cos(225*PI/180);
					s_desti.y = radius * sin(225*PI/180) + 2200;
				}
		}
		case 24:
		{
			if(left_right==RIGHT)
			{
				s_desti.x = radius * cos(135*PI/180);
				s_desti.y = radius * sin(135*PI/180) + 2200;
			}
			else if(left_right==LEFT)
			{
				s_desti.x = radius * cos(-45*PI/180);
				s_desti.y = radius * sin(-45*PI/180) + 2200;
			}
		}
		default:
			break;
	}
	return s_desti;
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
	if(direction==2)
	{
		for(int i=0;i<= (point_num/3-1);i++)
		{
			s_route->x[i] = radius_1 * cos(PI-2*PI/(point_num/3)*(i+1-point_num/12));
			s_route->y[i] = radius_1 * sin(PI-2*PI/(point_num/3)*(i+1-point_num/12))+2200;
			s_route->angle[i] = 180-360/(point_num/3)*(i-point_num/24) -90;
		}
		for(int i=point_num/3;i<= (2*point_num/3-1);i++)
		{
			s_route->x[i] = radius_2 * cos(PI-2*PI/(point_num/3)*(i-point_num/24));
			s_route->y[i] = radius_2 * sin(PI-2*PI/(point_num/3)*(i-point_num/24))+2200;
			s_route->angle[i] = 180-360/(point_num/3)*(i-point_num/24) -90;
		}
		for(int i=2*point_num/3;i<= (point_num-1);i++)
		{
			s_route->x[i] = radius_3 * cos(PI-2*PI/(point_num/3)*(i-point_num/24));
			s_route->y[i] = radius_3 * sin(PI-2*PI/(point_num/3)*(i-point_num/24))+2200;
			s_route->angle[i] = 180-360/(point_num/3)*(i-point_num/24) -90;
		}
	}
	else if(direction==1)
	{
		for(int i=0;i<= (point_num/3-1);i++)
		{
			s_route->x[i] = radius_1 * cos(2*PI/(point_num/3)*(i-point_num/24));
			s_route->y[i] = radius_1 * sin(2*PI/(point_num/3)*(i-point_num/24))+2200;
			s_route->angle[i] = 360/(point_num/3)*(i-point_num/24) +90;
		}
		for(int i=point_num/3;i<= (2*point_num/3-1);i++)
		{
			s_route->x[i] = radius_2 * cos(2*PI/(point_num/3)*(i-point_num/24));
			s_route->y[i] = radius_2 * sin(2*PI/(point_num/3)*(i-point_num/24))+2200;
			s_route->angle[i] = 360/(point_num/3)*(i-point_num/24)+90 ;
		}
		for(int i=2*point_num/3;i<= (point_num-1);i++)
		{
			s_route->x[i] = radius_3 * cos(2*PI/(point_num/3)*(i-point_num/24));
			s_route->y[i] = radius_3 * sin(2*PI/(point_num/3)*(i-point_num/24))+2200;
			s_route->angle[i] = 360/(point_num/3)*(i-point_num/24)+90 ;
		}
	}
	for(int j=0;j<=(point_num-1);j++)
	{
		s_route->angle[j] -= 90;
		if (s_route->angle[j] >  180.0f)  s_route->angle[j] -= 360.0f;
		if (s_route->angle[j] < -180.0f)  s_route->angle[j] += 360.0f;
	}
}
/**
 * @brief design every point of route 以出发点为原点计算路径上各点的x,y坐标
 * @param direction 向左为2向右为1 point_num 点的个数 alpha 初始半径 beta 
 * @return None
 * @attention None
 */
void design_point_of_helix_route(struct route_point *s_route,int direction,int point_num,
	int alpha,int beta,float circle_num)
{
	static float radius;
	static float theta;
	if(direction==2)
	{
		for(int i=0;i<= (point_num-1);i++)
		{
			theta = PI-2*PI/(point_num/circle_num)*(i-point_num/24);
			radius = alpha - beta * theta;
			s_route->x[i] = radius * cos(theta);
			s_route->y[i] = radius * sin(theta) + 2200;
			s_route->angle[i] = 180-360/(point_num/circle_num)*(i-point_num/24) -90;
		}	
	}
	else if(direction==1)
	{
		for(int i=0;i<= (point_num-1);i++)
		{
			theta = 2*PI/(point_num/circle_num)*(i-point_num/24);
			radius = alpha + beta * theta;
			s_route->x[i] = radius * cos(theta);
			s_route->y[i] = radius * sin(theta) + 2200;
			s_route->angle[i] = 360/(point_num/circle_num)*(i-point_num/24) +90;
		}
	}
	for(int j=0;j<=(point_num-1);j++)
	{
		s_route->angle[j] -= 90;
		if (s_route->angle[j] >  180.0f)  s_route->angle[j] -= 360.0f;
		if (s_route->angle[j] < -180.0f)  s_route->angle[j] += 360.0f;
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
			for(int i=*point_addr-4;i<=*point_addr+4;i++)
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
//	printf("aim x %d y %d ang %.2f\r\n",s_route->x[*point_addr],s_route->y[*point_addr],s_route->angle[*point_addr]);
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
	int aim_point_x,int aim_point_y,float aim_point_angle,int pos_x,int pos_y,float pos_angle,int speed,int jam_back_time,
		struct s_motor_data *s_left,struct s_motor_data *s_right)
{
	static ActLine2 now_point={0};
	static ActLine2 aim_point={0};
	static int jam_back_counter=0;
	static float aim_angle,aim_ang_tol,angle_out;
	now_point.point.x = pos_x;
	now_point.point.y = pos_y;
	now_point.angle = pos_angle;
	aim_point.point.x = aim_point_x;
	aim_point.point.y = aim_point_y;
	aim_point.angle = aim_point_angle;
	aim_angle = MvByLine(now_point,aim_point) - 90;
	aim_ang_tol = pos_angle + CcltAngleSub(aim_angle,pos_angle);
	if(angle_pid_debug==1)
	{
		pid_struct_init(s_ang_pid,V1,200,P,I,D);
	}
	angle_out = pid_calculate(s_ang_pid,pos_angle,aim_ang_tol);
//	printf("aimangle %.2f\r\n",aim_angle);
//	printf("aimmidangle %.2f\r\n",aim_ang_tol);
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
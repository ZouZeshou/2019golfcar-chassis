#include "chassis.h"
#include "pid.h"
#include "STMGood.h"
#include "math.h"
struct s_motor_data s_trans_motor={0};
struct s_motor_data s_leftmotor={0};
struct s_motor_data s_rightmotor={0};
struct pid s_leftmotor_pid;
struct pid s_rightmotor_pid;
struct pid s_trans_pos_pid;
struct pid s_trans_spd_pid;
#define TRAVEL 147456  //4096*36=147456
#define ENCODE_ANGLE 0.0439506776 //360/8191
#define RPM_DPS 6 //1/60*360
int trans_pid_debug=0;
int trans_motor_jam=0;
int step1_finish = 0;
int64_t step1_position=0;
/**
 * @brief Initialize of chassis
 * @param None
 * @return None
 * @attention None
 */
void chassis_para_init(void)
{
	pid_struct_init(&s_leftmotor_pid,8000,2000,5,0.1,0);
	pid_struct_init(&s_rightmotor_pid,8000,2000,5,0.1,0);
	pid_struct_init(&s_trans_pos_pid,500,100,20,0,0);
	pid_struct_init(&s_trans_spd_pid,8000,3000,30,0,0);
	s_trans_motor.target_pos = s_trans_motor.back_position;
}
/**
 * @brief deal the dicrete encode to continue data
 * @param None
 * @return None
 * @attention None
 */
void continue_motor_pos(struct s_motor_data *s_motor)
{
	if(s_motor->back_position - s_motor->back_pos_last > 5000)
	{
		s_motor->circle_num--;
	}
	else if(s_motor->back_position - s_motor->back_pos_last < -5000)
	{
		s_motor->circle_num++;
	}
	s_motor->tol_pos = s_motor->back_position +s_motor->circle_num * 8191;
}
/**
 * @brief transmit a ball
* @param direction:拨球的方向 -1为向上 1为向下
 * @return None
 * @attention None
 */
void transmit_a_ball(int direction,struct s_motor_data *s_motor)
{
	if(trans_motor_jam==0)
	{
		if(direction==1)
			s_motor->target_pos += TRAVEL;
		else if(direction==-1)
			s_motor->target_pos -= TRAVEL;
	}
}
/**
 * @brief transmit a ball by step
* @param 
 * @return None
 * @attention None
 */
void transmit_a_ball_by_step_a(struct s_motor_data *s_motor,float step1_size,int time_out)
{
	static int count = 0;
	if(trans_motor_jam==0)
	{
		if(step1_finish==0)
		{
			if(count++ >= time_out)
			{
				s_motor->target_pos += (int)(TRAVEL * step1_size);
				step1_finish = 1;
				count = 0;
			}
		}
		else
		{
		count = 0;
		}
	}
}
void transmit_a_ball_by_step_b(struct s_motor_data *s_motor,float step2_size,int time_out)
{
		static int count=0;
		if(trans_motor_jam==0)
		{
			if(step1_finish==1)
			{
				if(count++>=time_out)
				{
					step1_finish = 0;
					s_motor->target_pos += (int)(TRAVEL * step2_size);
					count =0;
				}
			}
			else
			{
				count = 0;
			}
		}
}
/**
 * @brief 检测卡球,如果卡球，反转
* @param s_motor_data
 * @return None
 * @attention None
 */
void deal_motor_jam(struct s_motor_data *s_motor,int time_out)
{
	static int jam_counter=0;
	if(abs(s_motor->target_pos - s_motor->tol_pos)>=5000)
	{
		if(jam_counter++>=time_out)
		{
			jam_counter = 0;
			s_motor->target_pos += TRAVEL;
		}
	}
	else
	{
		jam_counter = 0;
	}
	
}
/**
 * @brief calculate the current of trans_motor
 * @param None
 * @return None
 * @attention None
 */
void calculate_trans_current(struct s_motor_data *s_motor,struct pid *s_pos_pid,struct pid *s_spd_pid)
{
	if(trans_pid_debug)
	{
		pid_struct_init(&s_trans_pos_pid,V1,100,P,I,D);
		pid_struct_init(&s_trans_spd_pid,V2,4000,p,i,d);
	}
	s_motor->target_speed = pid_calculate(s_pos_pid,s_motor->tol_pos*ENCODE_ANGLE/36,s_motor->target_pos*ENCODE_ANGLE/36);
	s_motor->out_current = pid_calculate(s_spd_pid,s_motor->back_speed*RPM_DPS/36,s_motor->target_speed);
}
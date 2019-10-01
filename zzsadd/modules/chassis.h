#ifndef __CHASSIS_H
#define __CHASSIS_H
#include "stm32f4xx.h"
#include "pid.h"
struct s_motor_data 
{
	int16_t back_speed;
	int16_t back_position;
	int16_t back_pos_last;
	int circle_num;
	int target_speed;
	int64_t tol_pos;
	int64_t target_pos;
	int out_current;
};

void chassis_para_init(void);
void continue_motor_pos(struct s_motor_data *s_motor);
void deal_motor_jam(struct s_motor_data *s_motor,int time_out);
void transmit_a_ball(int direction,struct s_motor_data *s_motor);
void calculate_trans_current(struct s_motor_data *s_motor,struct pid *s_pos_pid,struct pid *s_spd_pid);
extern struct pid s_leftmotor_pid;
extern struct pid s_rightmotor_pid;
extern struct pid s_trans_pos_pid;
extern struct pid s_trans_spd_pid;
extern struct s_motor_data s_leftmotor;
extern struct s_motor_data s_rightmotor;
extern struct s_motor_data s_trans_motor;
extern int trans_motor_jam;
#endif

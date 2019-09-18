#ifndef __CHASSIS_H
#define __CHASSIS_H
#include "stm32f4xx.h"
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
extern struct pid s_leftmotor_pid;
extern struct pid s_rightmotor_pid;
extern struct s_motor_data s_leftmotor;
extern struct s_motor_data s_rightmotor;
extern struct s_motor_data s_trans_motor;
#endif

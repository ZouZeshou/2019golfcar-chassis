#include "chassis.h"
#include "pid.h"
struct s_motor_data s_trans_motor={0};
struct s_motor_data s_leftmotor={0};
struct s_motor_data s_rightmotor={0};
struct pid s_leftmotor_pid;
struct pid s_rightmotor_pid;
struct pid s_trans_pos_pid;
struct pid s_trans_spd_pid;
#define TRAVEL 10000
#define ENCODE_ANGLE 0.0439506776 //360/8191
#define RPM_DPS 6 //1/60*360
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
	pid_struct_init(&s_trans_pos_pid,400,100,0,0,0);
	pid_struct_init(&s_trans_spd_pid,8000,2000,0,0,0);
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
 * @param None
 * @return None
 * @attention None
 */
void transmit_a_ball(struct s_motor_data *s_motor)
{
	s_motor->target_pos += TRAVEL;
}
/**
 * @brief calculate the current of trans_motor
 * @param None
 * @return None
 * @attention None
 */
void calculate_trans_current(struct s_motor_data *s_motor,struct pid *s_pos_pid,struct pid *s_spd_pid)
{
	s_motor->target_speed = pid_calculate(s_pos_pid,s_motor->tol_pos*ENCODE_ANGLE,s_motor->target_pos*ENCODE_ANGLE);
	s_motor->out_current = pid_calculate(s_spd_pid,s_motor->back_speed*RPM_DPS,s_motor->target_speed*RPM_DPS);
}
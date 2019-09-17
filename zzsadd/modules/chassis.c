#include "chassis.h"
#include "pid.h"
struct s_motor_data s_trans_motor;
struct s_motor_data s_leftmotor;
struct s_motor_data s_rightmotor;
struct pid s_leftmotor_pid;
struct pid s_rightmotor_pid;
/**
 * @brief Initialize of chassis
 * @param None
 * @return None
 * @attention None
 */
void chassis_para_init(void)
{
	pid_struct_init(&s_leftmotor_pid,8000,2000,0,0,0);
	pid_struct_init(&s_rightmotor_pid,8000,2000,0,0,0);
}
/**
 * @brief keep a speed
 * @param None
 * @return None
 * @attention None
 */
void get_current_from_speed(struct pid *s_pid,float get,float set)
{
	pid_calculate(s_pid,get,set);
}
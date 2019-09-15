#include "chassis.h"
#include "pid.h"
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
	pid_struct_init(&s_leftmotor_pid,float maxout,float inte_limit,float kp,float ki,float kd);
}
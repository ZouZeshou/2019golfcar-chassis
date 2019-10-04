#include "usr_task.h"
#include "freertos.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "drv_robotservo.h"
#include "STMGood.h"
#include "drv_uart.h"
#include "usart.h"
#include "drv_can.h"
#include "can.h"
#include "chassis.h"
#include "drv_uart.h"
#include "drv_locationsystem.h"
#include "route.h"
#include "math.h"
#include "drv_coloursensor.h"
#include "chassis.h"
#include "pid.h"
#include "drv_io.h"
#include "protocol.h"
static int init_ok;
static int start_signal = 1;
static int calculate_init;
int step = 2;
int point_num = 48;
int now_point = 0; 
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
void StartTask02(void const * argument)
{
	static int color_count=0;
  for(;;)
  {
		if(init_ok)
		{
			deal_motor_jam(&s_trans_motor,1000/2);
			calculate_trans_current(&s_trans_motor,&s_trans_pos_pid,&s_trans_spd_pid);
			s_send_data.ball_color=detect_the_color(&s_color_data);
//			if(s_color_data.ball_color==s_color_data.ball_color_last)
//			{
//				if(color_count++>=10)
//				{
//					s_send_data.ball_color = s_color_data.ball_color;
//					color_count =0;
//				}
//			}
//			else
//			{
//				color_count = 0;
//			}
			
			if((s_receive_data.start_run==1||s_receive_data.start_run==2) && step == 0)
			{
				step=1;
			}
			switch(step)
			{
					case 1://��Ȧ
					{
							if(calculate_init==0)
							{
								//design_point_of_route(&s_route,1,point_num,900,1300,1600);
								design_point_of_helix_route(&s_route,s_receive_data.start_run,point_num,950,25);
								calculate_init = 1;
							}
							else
							{
								update_point(&s_route,&now_point,s_posture.pos_x,s_posture.pos_y,400,3000/2,point_num);
								calculate_motor_current(&s_leftmotor_pid,&s_rightmotor_pid,&s_angle_pid,s_route.x[now_point],
									s_route.y[now_point],s_route.angle[now_point],s_posture.pos_x,s_posture.pos_y,s_posture.zangle,6000,800/2,&s_leftmotor,&s_rightmotor);
							}
							if(now_point>=point_num)
							{
								step++;
							}
							break;
					}
					case 2:
					{
							s_send_data.finish_run = 1;
							s_leftmotor.target_speed = 0;
							s_rightmotor.target_speed = 0;
					//		s_leftmotor.out_current = (int)(pid_calculate(&s_leftmotor_pid,s_leftmotor.back_speed,s_leftmotor.target_speed));
					//		s_rightmotor.out_current = (int)(pid_calculate(&s_rightmotor_pid,s_rightmotor.back_speed,s_rightmotor.target_speed));
							
						break;
					}
					default:
						break;
			}
		}
    osDelay(2);
  }
}
void StartTask06(void const * argument)
{
	static int shoot_count=0;
  for(;;)
  {
		if(abs(s_trans_motor.target_pos - s_trans_motor.tol_pos)<=3000)
		{
				s_send_data.colorsensor_ready = 1;
				switch(s_send_data.ball_color)
								{
									case BLACK:
									{
										if(s_receive_data.ready_to_shoot==1&&s_receive_data.ready_to_shoot_last==0)
											transmit_a_ball(-1,&s_trans_motor);
//										else if(s_receive_data.black_or_white==1)
//											transmit_a_ball(1,&s_trans_motor);
										break;
									}
									case WHITE:
									{
										if(s_receive_data.ready_to_shoot==1&&s_receive_data.ready_to_shoot_last==0)
											transmit_a_ball(-1,&s_trans_motor);
//										else if(s_receive_data.black_or_white==0)
//											transmit_a_ball(1,&s_trans_motor);
										break;
									}
									case PINK:
									{
										if(s_receive_data.ready_to_shoot==1 && s_receive_data.ready_to_shoot_last==0)
											transmit_a_ball(-1,&s_trans_motor);
										break;
									}
									case ENVIRONMENT:
									{
										if(shoot_count++>=1000)
										{
												shoot_count = 0;
												transmit_a_ball(-1,&s_trans_motor);
										}
										break;
									}
									default:
										break;
								}
		}
		else
		{
			s_send_data.colorsensor_ready = 0;
		}
    osDelay(2);
  }
}
void StartTask07(void const * argument)
{
  for(;;)
  {
		if(abs(s_trans_motor.target_pos - s_trans_motor.tol_pos)<=3000)
		{
				//transmit_a_ball_by_step_b(&s_trans_motor,0.4,1000/50);		
		}
    osDelay(50);
  }
}
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
void StartTask03(void const * argument)
{
  for(;;)
  {
		if(calculate_init)
		{
			Can_SendMsg(&hcan1,0x200,s_leftmotor.out_current,s_rightmotor.out_current,s_trans_motor.out_current,0);
		}
		Can_SendMsg(&hcan1,0x200,s_leftmotor.out_current,s_rightmotor.out_current,s_trans_motor.out_current,0);
    osDelay(2);
  }
}
/**
* @brief initial function
* @param argument: Not used
* @retval None
*/
void StartTask04(void const * argument)
{
	static int init_counter;
  for(;;)
  {
		s_send_data.pos_x.f = s_posture.pos_x;
		s_send_data.pos_y.f = s_posture.pos_y;
		s_send_data.angle.f = s_posture.zangle;
		send_data_to_gimbal(&huart4);
		if(init_ok==0)
		{
			chassis_para_init();
			route_init();
			if(init_counter++ >= 400)
			{
				init_ok = 1;
				PWM1 = 1250;
				PWM2 = 1250;
			}
		}
    osDelay(40);
  }
}
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
void StartTask05(void const * argument)
{
  for(;;)
  {
		static int angle;
		if(init_ok)
		{
			if(trans_motor_jam==0)
			{
				HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_7);
				HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_14);
			}
			printf("type %d lux %d ct%d color %d\r\n",s_color_data.Start,s_color_data.Lux,s_color_data.CT,s_color_data.color);
			printf("ballcolor %d\r\n",s_send_data.ball_color);
//			for(int i=0;i<72;i++)
//			{
//				printf("num %d x %d y %d \r\n",i,s_route.x[i],s_route.y[i]);
//			}
//			printf("atan %.2f\r\n",atan2f(P,I)*180/3.14);
			printf("rightpos %d spd %d\r\n",s_rightmotor.back_position,s_rightmotor.back_speed);
			printf("leftpos %d spd %d\r\n",s_leftmotor.back_position,s_leftmotor.back_speed);
			printf("trans spd %d pos %lld target %lld\r\n",s_trans_motor.back_speed,s_trans_motor.tol_pos,s_trans_motor.target_pos);
			printf("pospid err %.2f out %.2f spdpid err %.2f out %.2f\r\n",s_trans_pos_pid.err,s_trans_pos_pid.out,s_trans_spd_pid.err,s_trans_spd_pid.out);
//			printf("targetspad %d %d\r\n",s_leftmotor.target_speed,s_rightmotor.target_speed);
			printf("now_point %d\r\n",now_point);
			printf("step %d\r\n",step);
			printf("receive B_W %d ready %d last %d start %d\r\n",s_receive_data.black_or_white,s_receive_data.ready_to_shoot,s_receive_data.ready_to_shoot_last,s_receive_data.start_run);
			printf("ang %.2f x %.2f y %.2f \r\n",s_posture.zangle,s_posture.pos_x,s_posture.pos_y);
//			printf("current %d %d \r\n",s_leftmotor.out_current,s_rightmotor.out_current);
//		transmit_a_ball(1,&s_trans_motor);
//			transmit_a_ball_by_step_a(&s_trans_motor,0.6,1000/200);
		}
    osDelay(200);
  }
}

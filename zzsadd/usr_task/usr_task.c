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
#include "detect.h"
#include "ANO_DT.h"
#include "drv_imu.h"
static int init_ok;
//static int start_signal = 1;
static int calculate_init;
int step = 0;
int point_num = 72;
int now_point = 0; 
int trans_motor_off = 0;
int start_next_path_count = 0;
int start_next_step = 0;
int calculate_path = 0;
 float circle_num = 0;
 int ball_color=0;
 int finish_run_cnt=0;
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
void StartTask02(void const * argument)
{

  for(;;)
  {
		if(init_ok)
		{		
			if(((s_receive_data.start_run==1||s_receive_data.start_run==2)&&(step==0))||start_next_step==1)
			{
				step = 1;
				start_next_step = 0;
				calculate_init = 0;
				now_point = 0;
				s_send_data.finish_run = 0;
				finish_run_cnt = 0;
				ramp_init(&s_left_ramp);
				ramp_init(&s_right_ramp);
			}
			switch(step)
			{
					case 1://跑圈
					{
							if(calculate_init==0)
							{
								switch(calculate_path)
								{
									case 0 :
									{
										circle_num = choose_detination_by_circle(s_receive_data.start_run,s_receive_data.black_or_white,s_receive_data.bucket_num)-1.2 ;
										design_point_of_helix_route(&s_route,s_receive_data.start_run,point_num,850,20,circle_num,1300,800,1300);
										calculate_path++;
										break;
									}
									case 1:
									{
										design_point_of_helix_route(&s_route,s_receive_data.start_run,point_num,1500,0,circle_num,2000,900,1300);
										now_point = (circle_num - 1) * point_num/circle_num + point_num/12;
										calculate_path++;
										break;
									}
									case 2:
									{
										design_point_of_helix_route(&s_route,s_receive_data.start_run,point_num,1800,0,circle_num,1900,800,1300);
										now_point = (circle_num - 1) * point_num/circle_num + point_num/12;
										calculate_path++;
										break;
									}
									case 3:
									{
										design_point_of_helix_route(&s_route,s_receive_data.start_run,point_num,800,0,circle_num,1900,800,1300);
										now_point = (circle_num - 1) * point_num/circle_num + point_num/12;
										calculate_path++;
										break;
									}
									case 4:
									{
										design_point_of_helix_route(&s_route,s_receive_data.start_run,point_num,1200,0,circle_num,1900,800,1300);
										now_point = (circle_num - 1) * point_num/circle_num + point_num/12;
										calculate_path++;
										break;
									}
									case 5:
									{
										design_point_of_helix_route(&s_route,s_receive_data.start_run,point_num,1500,-20,circle_num,1900,800,1300);
										now_point = (circle_num - 1) * point_num/circle_num + point_num/12;
										calculate_path = 1;
										break;
									}
									default:
										break;
								}
								calculate_init = 1;
							}
							else
							{
								update_point(&s_route,&now_point,s_posture.pos_x,s_posture.pos_y,400,1400/2,point_num);
								calculate_motor_current(&s_leftmotor_pid,&s_rightmotor_pid,&s_angle_pid,s_route.x[now_point],
									s_route.y[now_point],s_route.angle[now_point],s_posture.pos_x,s_posture.pos_y,s_posture.zangle,6000,400/2,&s_leftmotor,&s_rightmotor);
							}
							if(now_point>=point_num)
							{
								step++;
							}
							break;
					}
					case 2:
					{
							if(finish_run_cnt++>=400)
							{
								s_send_data.finish_run = 1;
								finish_run_cnt = 0;
							}
							s_leftmotor.target_speed = 0;
							s_rightmotor.target_speed = 0;
							s_leftmotor.out_current = (int)(pid_calculate(&s_leftmotor_pid,s_leftmotor.back_speed,s_leftmotor.target_speed));
							s_rightmotor.out_current = (int)(pid_calculate(&s_rightmotor_pid,s_rightmotor.back_speed,s_rightmotor.target_speed));					
							break;
					}
					default:
						break;
			}
		}
    osDelay(2);
  }
}
/**
* @brief 
* @param argument: Not used
* @retval None
*/
void StartTask03(void const * argument)
{
  static int shoot_count=0;
	static int black_cnt = 0;
	static int white_cnt = 0;
  for(;;)
  {
	  if(init_ok)
	  {
			deal_motor_jam(&s_trans_motor,1500/2);
			calculate_trans_current(&s_trans_motor,&s_trans_pos_pid,&s_trans_spd_pid);
			ball_color = detect_the_color(&s_color_data);
			gimbal_data_state = JudgeDeviceState(gimbal_data_fps,4);
			
			if((abs(s_trans_motor.target_pos - s_trans_motor.tol_pos)<=2000))
			{
				s_send_data.colorsensor_ready = 1;
				
				switch(ball_color)
				{
					case BLACK:
					{
						shoot_count = 0;
						white_cnt = 0;
						s_send_data.ball_color = BLACK;
						if(s_receive_data.black_or_white == WHITE && s_send_data.finish_run==0)
						{
							if(black_cnt++ >= 5)
							{
								transmit_a_ball(-1,&s_trans_motor);
								black_cnt = 0;
							}
						}
						else if	(gimbal_data_state==ONLINE&&s_receive_data.ready_to_shoot==1&&s_receive_data.ready_to_shoot_last==0)
						{
							black_cnt = 0;
							transmit_a_ball(-1,&s_trans_motor);
						}
						else
						{
							black_cnt = 0;
						}
						break;
					}
					case WHITE:
					{
						black_cnt = 0;
						shoot_count = 0;
						s_send_data.ball_color = WHITE;
						if(s_receive_data.black_or_white == BLACK && s_send_data.finish_run==0)
						{
							if(white_cnt++ >= 5)
							{
								transmit_a_ball(-1,&s_trans_motor);
								white_cnt = 0;
							}
						}
						else if	(gimbal_data_state==ONLINE&&s_receive_data.ready_to_shoot==1&&s_receive_data.ready_to_shoot_last==0)
						{
							white_cnt = 0;
							transmit_a_ball(-1,&s_trans_motor);
						}
						else
						{
							white_cnt = 0;
						}
						break;
					}
					case PINK:
					{
						black_cnt = 0;
						white_cnt = 0;
						shoot_count = 0;
						s_send_data.ball_color = PINK;
						if(gimbal_data_state==ONLINE&&s_receive_data.ready_to_shoot==1 && s_receive_data.ready_to_shoot_last==0)
						{
							transmit_a_ball(-1,&s_trans_motor);
						}
						break;
					}
					case ENVIRONMENT:
					{
						black_cnt = 0;
						white_cnt = 0;
						s_send_data.ball_color = ENVIRONMENT;
						if(shoot_count++>=60)
						{
							shoot_count = 0;
							if(s_send_data.finish_run==1)
							{
								transmit_a_ball(-1,&s_trans_motor);
								if(start_next_path_count++ > 2)
								{
									start_next_step = 1;
									start_next_path_count = 0;
								}
							}
							else
							{
								start_next_path_count = 0;
							}
						}
						break;
					}
					default:
						shoot_count = 0;
						break;
				}
			}
			else
			{
				//对标志位进行重置
				s_send_data.colorsensor_ready = 0;
			}

	}
    osDelay(10);
  }
}

/**
* @brief 
* @param argument: Not used
* @retval None
*/
void StartTask05(void const * argument)
{
  for(;;)
  {
	mpu_get_data(&sensor);
	UpdateIMU(&sensor);
//		mahony_ahrs_update(&sensor,&atti);
//	imu_temp_keep();

    osDelay(1);
  }
}
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
void StartTask04(void const * argument)
{
  for(;;)
  {
//	  if(trans_motor_jam == 1)
//	  {
//		s_trans_motor.out_current = 0;
//	  }
	  if(calculate_init)
	  {
			Can_SendMsg(&hcan1,0x200,s_leftmotor.out_current,s_rightmotor.out_current,s_trans_motor.out_current,0);
	  }
//		Can_SendMsg(&hcan1,0x200,0,0,s_trans_motor.out_current,0);
    osDelay(2);
  }
}
/**
* @brief initial function
* @param argument: Not used
* @retval None
*/
void StartTask07(void const * argument)
{
	static int init_counter;
	static float zang_buff;
  for(;;)
  {
		s_send_data.pos_x.f = s_posture.pos_x;
		s_send_data.pos_y.f = s_posture.pos_y;
		yaw_angle_buff -= (yaw_angle_buff - ((int)(yaw_angle_buff)%(360)));
		if(yaw_angle_buff<0)
		{
			yaw_angle_buff  += 360;
		}
		s_send_data.angle.f = yaw_angle_buff;
		send_data_to_gimbal(&huart4);
		zang_buff = s_send_data.angle.f;
		if(zang_buff > 180)
		{
			zang_buff -= 360; 
		}
		s_posture.zangle = zang_buff;
		if(init_ok==0)
		{
			chassis_para_init();
			route_init();
			if(init_counter++ >= 400)
			{	
				PWM1 = 1220;
				PWM2 = 1220;
			}
			if(init_counter>=700)
			{
				init_ok = 1;
				PWM1 = 1280;
				PWM2 = 1280;
			}
		}

//		ANO_DT_Data_Exchange();
    osDelay(5);
  }
}
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
void StartTask06(void const * argument)
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
//			printf("back_count %d \r\n",back_count);
//			printf("Offline a %d b %d c %d\r\n",Offline[0],Offline[1],Offline[2]);
//			for(int i=0;i<48;i++)
//			{
//				printf("num %d x %d y %d \r\n",i,s_route.x[i],s_route.y[i]);
//			}
//			printf("atan %.2f\r\n",atan2f(P,I)*180/3.14);
//			printf("rightpos %d spd %d\r\n",s_rightmotor.back_position,s_rightmotor.back_speed);
//			printf("leftpos %d spd %d\r\n",s_leftmotor.back_position,s_leftmotor.back_speed);
//			printf("trans spd %d pos %lld target %lld\r\n",s_trans_motor.back_speed,s_trans_motor.tol_pos,s_trans_motor.target_pos);
//			printf("pospid err %.2f out %.2f spdpid err %.2f out %.2f\r\n",s_trans_pos_pid.err,s_trans_pos_pid.out,s_trans_spd_pid.err,s_trans_spd_pid.out);
//			printf("targetspad %d %d\r\n",s_leftmotor.target_speed,s_rightmotor.target_speed);
//			printf("circle_num %.2f\r\n",circle_num);
			printf("now_point %d\r\n",now_point);
			printf("step %d\r\n",step);
			printf("receive B_W %d ready %d last %d start %d bucket %d\r\n",s_receive_data.black_or_white,s_receive_data.ready_to_shoot,s_receive_data.ready_to_shoot_last,s_receive_data.start_run,s_receive_data.bucket_num);
			printf("ang %.2f x %.2f y %.2f \r\n",s_posture.zangle,s_posture.pos_x,s_posture.pos_y);
			printf("ang_board %.2f\r\n",s_send_data.angle.f);
//			printf("current %d %d \r\n",s_leftmotor.out_current,s_rightmotor.out_current);
//		    transmit_a_ball(-1,&s_trans_motor);
//			transmit_a_ball_by_step_a(&s_trans_motor,0.6,1000/200);
//			calibrate_yaw_angle(s_posture.zangle);
		}
    osDelay(200);
  }
}

#include "protocol.h"
#include "CRC.h"
#include "drv_uart.h"
#include "usart.h"
#include "detect.h"
struct data_to_send s_send_data={0};
struct data_receive s_receive_data={0};
int64_t gimbal_data_fps=0;
int gimbal_data_state = 0;
/**
 * @brief get the data from gimbal
 * @param 
 * @return None
 * @attention None
 */
void deal_receive_data(uint8_t *buffer)
{
	if(buffer[0]==0xA5&&buffer[1]==0x5A&&Verify_CRC8_Check_Sum(buffer,8))
	{
		s_receive_data.start_run = buffer[2];
		s_receive_data.ready_to_shoot_last = s_receive_data.ready_to_shoot;
		s_receive_data.ready_to_shoot = buffer[3];
		s_receive_data.black_or_white = buffer[4];
		s_receive_data.bucket_num = buffer[5];
		gimbal_data_fps ++;
	}
}
/**
 * @brief send data to gimbal
 * @param 
 * @return None
 * @attention None
 */
void send_data_to_gimbal(UART_HandleTypeDef *huart)
{
	static uint8_t data[20]={0};
	data[0] = 0xA5;
	data[1] = 0x5A;
	
	data[2] = s_send_data.pos_x.u[0];
	data[3] = s_send_data.pos_x.u[1];
	data[4] = s_send_data.pos_x.u[2];
	data[5] = s_send_data.pos_x.u[3];
	
	data[6] = s_send_data.pos_y.u[0];
	data[7] = s_send_data.pos_y.u[1];
	data[8] = s_send_data.pos_y.u[2];
	data[9] = s_send_data.pos_y.u[3];
	
	data[10] = s_send_data.angle.u[0];
	data[11] = s_send_data.angle.u[1];
	data[12] = s_send_data.angle.u[2];
	data[13] = s_send_data.angle.u[3];
	
	data[14] = s_send_data.finish_run;
	data[15] = s_send_data.ball_color;
	data[16] = s_send_data.colorsensor_ready;
	
	Append_CRC8_Check_Sum(data,20);
	HAL_UART_Transmit(huart,data,20,0xff);
}
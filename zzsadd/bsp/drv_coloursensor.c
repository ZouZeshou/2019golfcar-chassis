#include "drv_coloursensor.h"
#include "drv_uart.h"
#include "usart.h"
struct s_colour_sensor_data s_color_data;
//
/**
 * @brief set the parameter of color sensor
 * @param None
 * @return None
 * @attention None
 */
void colour_sensor_init(void)
{
	uint8_t senddata[3];
	senddata[0]=0xA5;
	senddata[1]=0x82;
	senddata[2]=(senddata[0]+senddata[1]);
  USART_Send_Char(&huart2,senddata[0]);
	USART_Send_Char(&huart2,senddata[1]);
	USART_Send_Char(&huart2,senddata[2]);

}
/**
 * @brief get the sensor data from buffer zone
 * @param None
 * @return None
 * @attention None
 */
void deal_coloursensor_data(uint8_t * buffer)
{
	s_color_data.Start= buffer[2];
	s_color_data.Lux=  ((buffer[4]<<8)|buffer[5]);
	s_color_data.CT=   ((buffer[6]<<8)|buffer[7]);
	s_color_data.color=((buffer[8]<<8)|buffer[9]);
  s_color_data.END =  buffer[10];
}
/**
 * @brief detector the color of ball
 * @param None
 * @return None
 * @attention None
 */
int detect_the_color(struct s_colour_sensor_data *s_color)
{
	static int color_count=0;
	static int temp = 0;
	
	temp = s_color->ball_color;
	
	if((s_color->Lux>15&&s_color->Lux<100)&&(s_color->CT>4800&&s_color->CT<7000))
	{
		s_color->ball_color = BLACK;//15-40 5500-6700 57 5127 29 6133
	}
	else if((s_color->Lux>150&&s_color->Lux<800)&&(s_color->CT>4500&&s_color->CT<6000))
	{
		s_color->ball_color = WHITE;//100-250 4500-6000 610 5500 410 5300 140 5300
	}
	else if((s_color->Lux>100&&s_color->Lux<300)&&(s_color->CT>3400&&s_color->CT<4200))
	{
		s_color->ball_color = PINK;//50-120 3500-4200 223 3583 183 3600  234 3583
	}
	else
	{
		s_color->ball_color = ENVIRONMENT;//15-70 5000-7500 22 7000 19 7334 
	}
	
	if(temp == s_color->ball_color)
	{
		if(color_count++>=2)
		{
				color_count =0;
				s_color->ball_color_last = s_color->ball_color;
				return s_color->ball_color;
		}
		else
		{
			return s_color->ball_color_last;
		}
	}
	else
	{
		color_count = 0;
		return s_color->ball_color_last;
	}
	
}
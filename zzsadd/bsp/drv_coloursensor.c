#include "drv_coloursensor.h"
#include "drv_uart.h"
#include "usart.h"
struct s_colour_sensor_data s_color_data;
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
	senddata[1]=0x60;
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
	s_color_data.Start= buffer[0];
	s_color_data.Lux=  ((buffer[4]<<8)|buffer[5]);
	s_color_data.CT=   ((buffer[6]<<8)|buffer[7]);
	s_color_data.color=((buffer[8]<<8)|buffer[9]);
  s_color_data.END =  buffer[10];
}
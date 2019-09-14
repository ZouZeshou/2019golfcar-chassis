#include "drv_uart.h"
#include "usart.h"
#include "STMGood.h"
#include "stdio.h"
uint8_t Usart1buff[100]={0};
uint8_t Usart3buff[100]={0};
/**
 * @brief Enable USART1
 * @param None
 * @return None
 * @attention None
 */
void USART1_Enable(void)
{
	HAL_UART_Receive_IT(&huart1,Usart1buff,1);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_ERR);	
}
/**
* @brief usart send char
* @param argument: Not used
* @retval None
*/
void USART1_Send_Char(uint8_t u8_char)
{
		while((USART1->SR&0X40)==0); 
		USART1->DR = u8_char;
	
	//HAL_UART_Transmit_IT(&huart1,&u8_char,1);
}
/**
 * @brief Enable USART1
 * @param None
 * @return None
 * @attention None
 */
void USART3_Enable(void)
{
	HAL_UART_Receive_IT(&huart3,Usart3buff,1);
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_ERR);	
}
/**
 * @brief Error Callback function（串口中断回调函数）
 * @param None
 * @return None
 * @attention None
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart->ErrorCode == HAL_UART_ERROR_ORE)
	{
		__HAL_UART_CLEAR_OREFLAG(huart); //清除错误标志位，清空SR、DR寄存器
	}
}
/**
 * @brief Redirect function for printf（对printf函数的重定义）
 * @param None
 * @return None
 * @attention  The printf function could not be usedwithout this function
 */
int fputc(int ch, FILE *f)
{ 	
	while((USART3->SR&0X40)==0); 
	USART3->DR = (uint8_t) ch;      
	return ch;
}
/**
 * @brief rx callbackfunction  （串口接收中断回调函数）
 * @param None
 * @return None
 * @attention None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		Dealdata(Usart3buff[0]);//如果格式符合上位机的格式，则对相应变量进行赋值（P，I，D）
		__HAL_UART_CLEAR_PEFLAG(&huart3);//清除中断标志位
		HAL_UART_Receive_IT(&huart3,Usart3buff,1);//使能串口3
	}
}
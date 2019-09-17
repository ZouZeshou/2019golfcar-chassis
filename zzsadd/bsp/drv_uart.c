#include "drv_uart.h"
#include "usart.h"
#include "STMGood.h"
uint8_t Usart1buff[100]={0};
uint8_t Usart2buff[100]={0};
uint8_t Usart3buff[100]={0};
uint8_t Usart6buff[100]={0};
/**
 * @brief Enable USART
 * @param None
 * @return None
 * @attention None
 */
void USART_Enable(UART_HandleTypeDef *huart,uint8_t * buffer_addr)
{
	HAL_UART_Receive_IT(huart,buffer_addr,1);
	__HAL_UART_ENABLE_IT(huart,UART_IT_ERR);	
}
/**
* @brief usart send char
* @param argument: Not used
* @retval None
*/
void USART_Send_Char(UART_HandleTypeDef *huart,uint8_t u8_char)
{
		while((huart->Instance->SR&0X40)==0); 
		huart->Instance->DR = u8_char;
//	HAL_UART_Transmit_IT(&huart1,&u8_char,1);
}
/**
 * @brief Enable the Usart DMA
 * @param None
 * @return None
 * @attention  None
 */
void USART_DMA_Enable(UART_HandleTypeDef *huart,DMA_HandleTypeDef *hdma,uint8_t * buffer_addr,uint8_t data_size)
{
	 HAL_DMA_Start(hdma,(uint32_t)huart->Instance->DR, (uint32_t)buffer_addr,data_size);
	 huart->Instance->CR3 |= USART_CR3_DMAR;								/*!<DMA Enable Receiver         */
	 __HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);								/*!<使能串口的中断为空闲中断    */
	 HAL_UART_Receive_DMA(huart,buffer_addr,data_size);								/*!<DMA Receive data            */
	 __HAL_UART_ENABLE_IT(huart,UART_IT_ERR);								/*!<Enable Usart Error IT      	*/
}
/**
 * @brief Interrupt function for usart1
 * @param None
 * @return None
 * @attention None
 */
void USART_IDLE_IRQ(UART_HandleTypeDef *huart,uint8_t * buffer_addr,uint8_t data_size)
{
	
	if(__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET)
	{
			__HAL_UART_CLEAR_IDLEFLAG(huart);	//清除标志位和SR，DR寄存器
			HAL_UART_DMAStop(huart);
			HAL_UART_Receive_DMA(huart,buffer_addr,data_size);//函数中包括重新配置DMA
	}
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
		//printf("receive\r\n");
		__HAL_UART_CLEAR_PEFLAG(&huart3);//清除中断标志位
		HAL_UART_Receive_IT(&huart3,Usart3buff,1);//使能串口3
	}
}
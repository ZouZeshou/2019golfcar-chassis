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
 * @brief Error Callback function�������жϻص�������
 * @param None
 * @return None
 * @attention None
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart->ErrorCode == HAL_UART_ERROR_ORE)
	{
		__HAL_UART_CLEAR_OREFLAG(huart); //��������־λ�����SR��DR�Ĵ���
	}
}
/**
 * @brief Redirect function for printf����printf�������ض��壩
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
 * @brief rx callbackfunction  �����ڽ����жϻص�������
 * @param None
 * @return None
 * @attention None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		Dealdata(Usart3buff[0]);//�����ʽ������λ���ĸ�ʽ�������Ӧ�������и�ֵ��P��I��D��
		__HAL_UART_CLEAR_PEFLAG(&huart3);//����жϱ�־λ
		HAL_UART_Receive_IT(&huart3,Usart3buff,1);//ʹ�ܴ���3
	}
}
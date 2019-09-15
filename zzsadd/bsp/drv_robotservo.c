#include "drv_robotservo.h"
#include "usart.h"
#include "drv_uart.h"

//CDS5516的驱动层
/*
帧头   ID 长 指令 地址 数据.. 校验和
FF FF  01 04  03   03   00      F4
*/
/**
 * @brief 设置舵机的ID号为1
 * @param None
 * @return None
 * @attention None
 */
void Set_Num_1(void) //设置舵机的ID号为1
{
    USART1->CR1 &= ~0x4;
    USART_Send_Char(&huart1,0xFF);
    USART_Send_Char(&huart1,0XFF);
    USART_Send_Char(&huart1,0XFE);
    USART_Send_Char(&huart1,0X04);
    USART_Send_Char(&huart1,0X03);
    USART_Send_Char(&huart1,0X03);
    USART_Send_Char(&huart1,0X01);
    USART_Send_Char(&huart1,0XF6);
    USART1->CR1 |= 0x4;
}
/**
 * @brief 设置舵机旋转的目标角度
 * @param None
 * @return None
 * @attention None
 */
void Set_Num_Speed(uint8_t id,uint32_t arg)      //设置舵机旋转的目标角度
{
    static uint8_t i = 0; 
    uint8_t sum = 0x00; 
    uint8_t dat[] = {0xFF,0xFF,0,5,0x03,0x1E,0,0,0xFF};  //定义一个数据包
    dat[2] = id;                          //设置ID号
    dat[6] = (arg*0x3FF/300)&0xFF;        //设置目标角度的低字节
    dat[7] = (arg*0x3FF/300)>>8;          //设置目标角度的高字节
    for(i=2;i<8;i++)
    {
        sum+=dat[i];                    //计算校验和
    } 
    dat[8] = ~sum;                      
    
    for(i=0;i<9;i++)
    {
        USART_Send_Char(&huart1,dat[i]);       //将数据包发出
    }
}




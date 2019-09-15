#ifndef DRV_CAN__H
#define DRV_CAN__H
#include "stm32f4xx.h"

void Can_SendMsg(CAN_HandleTypeDef *hcan,uint32_t id,int16_t data1,int16_t data2,int16_t data3,int16_t data4);
void CANFilter_Enable(CAN_HandleTypeDef *hcan);
void CAN_Enable(CAN_HandleTypeDef *hcan);
#endif

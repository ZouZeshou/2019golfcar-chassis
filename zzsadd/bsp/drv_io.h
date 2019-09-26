#ifndef __DRV_IO_H
#define __DRV_IO_H
#include "tim.h"
#define PWM1 TIM4->CCR1
#define PWM2 TIM4->CCR2
void PWM_Init(TIM_HandleTypeDef *htim, uint32_t Channel);








#endif

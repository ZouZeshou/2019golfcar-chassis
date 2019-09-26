#include "drv_io.h"
//LED GREEN PF14 RED PE7
/**
 * @brief initialise the data will be used 
 * @param None
 * @return None
 * @attention None
 */
void PWM_Init(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	HAL_TIM_PWM_Start(htim,Channel);
	PWM1 = 1000;
	PWM2 = 1000;
}
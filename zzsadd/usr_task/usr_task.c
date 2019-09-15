#include "usr_task.h"
#include "freertos.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "drv_robotservo.h"
#include "STMGood.h"
#include "drv_uart.h"
#include "drv_can.h"
#include "can.h"
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
void StartTask02(void const * argument)
{
  for(;;)
  {
		static int angle;
		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_7);
		HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_14);
		if(angle==160)
		{
			angle = 82;
		}
		else
		{
			angle = 160;
		}
		
		Set_Num_Speed((uint8_t)0,(uint32_t)angle);
		printf("P %.1f I %.1f D %.1f\r\n",P,I,D);
    osDelay(1000);
  }
}
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
void StartTask03(void const * argument)
{
  for(;;)
  {
		Can_SendMsg(&hcan1,0x200,1000,500,0,0);
    osDelay(2);
  }
}

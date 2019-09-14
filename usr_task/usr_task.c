#include "usr_task.h"
#include "freertos.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "drv_robotservo.h"
#include "STMGood.h"
#include "drv_uart.h"
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
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
  /* USER CODE END StartTask02 */
}

#include "usr_task.h"
#include "freertos.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "drv_robotservo.h"
#include "STMGood.h"
#include "drv_uart.h"
#include "drv_can.h"
#include "can.h"
#include "chassis.h"
#include "drv_uart.h"
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
void StartTask02(void const * argument)
{
  for(;;)
  {

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
		Can_SendMsg(&hcan1,0x200,0,0,0,0);
    osDelay(2);
  }
}
/**
* @brief initial function
* @param argument: Not used
* @retval None
*/
void StartTask04(void const * argument)
{
	static int init_ok;
  for(;;)
  {
		if(init_ok==0)
		{
			chassis_para_init();
			init_ok = 1;
		}
    osDelay(5);
  }
}
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
void StartTask05(void const * argument)
{
  for(;;)
  {
				static int angle;
		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_7);
		HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_14);
		printf("rightpos %d spd %d\r\n",s_rightmotor.back_position,s_rightmotor.back_speed);
		printf("leftpos %d spd %d\r\n",s_leftmotor.back_position,s_leftmotor.back_speed);
//		if(angle==160)
//		{
//			angle = 82;
//		}
//		else
//		{
//			angle = 160;
//		}
//		
//		Set_Num_Speed((uint8_t)0,(uint32_t)angle);
//		printf("P %.1f I %.1f D %.1f\r\n",P,I,D);
    osDelay(50);
  }
}

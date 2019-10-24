#include "detect.h"

uint8_t Devicestate[13] = {0};
uint8_t Offline[13] = {0};
uint8_t Offline_last[13] = {0};
/*
	 Wheel_1;   0
	 Wheel_2;   1
	 trans_motor 2
*/
/**************zzs_add***************/
/**
 * @brief Judge the device online or offline
 * @param 
 * @return 
 * @attention  None
 */

void DeviceDetect(uint8_t *state,uint8_t *result,uint8_t time_out)
{
	static int counter[13] = {0};
	for(int i=0;i<13;i++)
	{
		if(state[i] == OFFLINE)
		{
			if(counter[i]++ > time_out && state[i] == OFFLINE)
			{
				result[i] = OFFLINE;
			}
		}
		else
		{
			counter[i] = 0;
			result[i] = ONLINE;
		}
	}
}

void GetDeviceState(void)
{	
	Devicestate[0] = JudgeDeviceState(s_fps.chassis_1,0);
	Devicestate[1] = JudgeDeviceState(s_fps.chassis_2,1);
	Devicestate[2] = JudgeDeviceState(s_fps.trans,2);
}

int JudgeDeviceState(int fps,int i)
{
	static int fpsold[13] = {0};
	static int fpschange[13] = {0};
	fpschange[i] = fps - fpsold[i];
	fpsold[i] = fps;
	if(fpschange[i] == 0)
		return OFFLINE;
	else
		return ONLINE;	
}


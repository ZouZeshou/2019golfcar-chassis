#include "drv_locationsystem.h"

struct posture_data s_posture={0};

/**
 * @brief get the locationsystem data
 * @param None
 * @return None
 * @attention None
 */
void get_loca_sys_data(uint8_t * buffer)
{
	static union
	{
		uint8_t data[24];
		float ActVal[6];
	}posture;
	
	if(buffer[0]==0x0D&&buffer[1]==0x0A)
	{
		for(int i=0;i<24;i++)
		{
			posture.data[i]=buffer[i+2];
		}
		s_posture.zangle=posture.ActVal[0];
		s_posture.xangle=posture.ActVal[1];
		s_posture.yangle=posture.ActVal[2];
		s_posture.pos_x =posture.ActVal[3];
		s_posture.pos_y =posture.ActVal[4];
		s_posture.w_z =posture.ActVal[5];
	}
}
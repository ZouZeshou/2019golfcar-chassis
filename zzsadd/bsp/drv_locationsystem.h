#ifndef __DRV_LOCATION_H
#define __DRV_LOCATION_H
#include "stm32f4xx.h"
//角度制转换为弧度制系数
#define CHANGE_TO_RADIAN    0.017453f   
//弧度制转换为角度制系数
#define CHANGE_TO_ANGLE     57.2958f
struct posture_data
{
	float pos_x;
	float pos_y;
	float zangle;
	float xangle;
	float yangle;
	float w_z;
	
	float ang_last;
	float ang_tol;
	int cir_num;
};
//点的结构体 单位mm
typedef struct
{
	float x;
	float y;
}ActPoint;


//点斜式结构体 ，斜率用角度制的角度代替
typedef struct
{
	ActPoint point;
	//角度制
	float    angle;
}ActLine2;


float CcltAngleSub(float minuend, float subtrahend);
float MvByLine(ActLine2 presentLine, ActLine2 targetLine);
//外部接口函数，电机速度控制

extern struct posture_data s_posture;


void get_loca_sys_data(uint8_t * buffer);
void angle_to_continue(struct posture_data *s_pos);
#endif

#include "drv_locationsystem.h"
#include "math.h"
struct posture_data s_posture={0};

/**
 * @brief get the locationsystem data
 * @param None
 * @return None
 * @attention None
 */
void get_loca_sys_data(uint8_t * buffer)
{
	static int init=0;
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
		s_posture.pos_x =-posture.ActVal[3];
		s_posture.pos_y =-posture.ActVal[4];
		s_posture.w_z =posture.ActVal[5];
	}

}
/**
 * @brief deal gyro data to continue
 * @param None
 * @return None
 * @attention None
 */
void angle_to_continue(struct posture_data *s_pos)
{
	if(s_pos->zangle-s_pos->ang_last<-300)
	{
		s_pos->cir_num = s_pos->cir_num + 1;
	}
	else if(s_pos->zangle-s_pos->ang_last>300)
	{
		s_pos->cir_num = s_pos->cir_num - 1;
	}
	s_pos->ang_tol = s_pos->zangle + s_pos->cir_num * 360;
}
/**
* @name 	CcltAngleAdd
* @brief	对-180,180交界处作处理
* @param	angle1:角度1;
			angle2:角度2;
* @retval
*/
static float CcltAngleAdd(float angle1, float angle2)
{
	float result = 0.0f;
	result = angle1 + angle2;
	if (result >  180.0f)  result -= 360.0f;
	if (result < -180.0f)  result += 360.0f;
	return result;
	
}
/**
* @name 	CcltAngleSub
* @brief	对-180,180交界处作处理
* @param	minuend: 被减数;
			subtrahend: 减数 A - B,A为被减数，B为减数;
* @retval
*/
float CcltAngleSub(float minuend, float subtrahend)
{
	float result = 0.0f;
	result = minuend - subtrahend;
	if (result >  180.0f)  result -= 360.0f;
	if (result < -180.0f)  result += 360.0f;
	return result;
}
/**
* @name 	CcltTwoLineIntersection2
* @brief	计算两条直线的交点
* @param	line1:直线1;
			line2:直线2;
* @retval
*/
static ActPoint CcltTwoLineIntersection2(ActLine2 line1, ActLine2 line2)
{
	ActPoint intersection;
	//斜率
	float k1 = 0.0f;
	float k2 = 0.0f;

	//因为浮点运算,未对与x轴垂直的直线处理。
	k1 = tan(line1.angle * CHANGE_TO_RADIAN);
	k2 = tan(line2.angle * CHANGE_TO_RADIAN);

	intersection.x = (line1.point.x*k1 - line1.point.y - line2.point.x * k2 + line2.point.y)
						 / (k1 - k2);
	intersection.y = k1 * (intersection.x - line1.point.x) + line1.point.y;

	return intersection;
}


/**
* @name 	CcltLineAngle
* @brief	计算两点直线方向角度
* @param	pointStart:起始点：
			pointEnd:终止点;
* @retval
*/
static float CcltLineAngle(ActPoint pointStart, ActPoint pointEnd)
{
	float a = 0.0f;
	float b = 0.0f;
	
	a = pointEnd.y - pointStart.y;
	b = pointEnd.x - pointStart.x;
	//atan2f范围可以包含-180到180  
	return (atan2f(a, b) * CHANGE_TO_ANGLE); 
}

static float s_angleErrOld = 0.0f; 
void Point2Ponit_Deinit()
{
	s_angleErrOld = 0.0f;
}

/**
  * @name 	MvByLine
  * @brief  	直线闭环，速度为正
  * @param  	presentLine：当前姿态；
				targetLine：v目标点目标方向，等同于目标直线；
				speed:直线速度;
  * @retval 	返回值为当前点到目标点距离。单位mm
  */
float MvByLine(ActLine2 presentLine, ActLine2 targetLine)
{
	//当前点到目标直线的垂线
	ActLine2 verticalLine;											
	//中点	
	ActPoint  midpoint;											
	//控制的目标角度
	float targetAngle = 0.0f;	
	//当前点到目标点距离	
	float psntToEndDis = 0.0f;																						  
	//目标直线加90度，求出垂线方向
	verticalLine.angle = CcltAngleAdd(targetLine.angle, 90.0f);
	//把当前点赋给垂线
	verticalLine.point = presentLine.point;		
	//计算垂线与目标直	
	midpoint = CcltTwoLineIntersection2(targetLine, verticalLine);
	//计算中点横坐标	
	midpoint.x = (midpoint.x + targetLine.point.x) * 0.5f;													  
	//计算中点纵坐标
	midpoint.y = (midpoint.y + targetLine.point.y) * 0.5f;													  
	//计算当前点到中点方向
	targetAngle = CcltLineAngle(presentLine.point, midpoint);												  
	//当前点到目标点距离
	return targetAngle;
}
/*******************************************************************************

  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name 	: path_tracking.c
  Author		: George 	
  Version		: V1.0.0	  
  Date			: 2017/07/21
  Description	: path_tracking 
  
  History:		  
				  
	1. Date 		:
	   Author		:
	   Modification :
	2. ...
	
*******************************************************************************/

/* includes *******************************************************************/
#include "motion_sense.h"
#include "global.h"
#include "usart_driver.h"
#include "motion_math.h"
#include "motion_types.h"

/* macros *********************************************************************/
/* static variables ***********************************************************/
static __inline void Tracking_Norm_2D(float* x, float* y)
{
	float length = sqrtf((*x)*(*x) + (*y)*(*y));
	if(length != 0)
	{
		*x = *x / length;
		*y = *y / length;
	}
}


// 更新位姿
void Motion_Get_imuData(T_motion_sensorData* mSensor)
{
	//extern float rot_vec[3];

}

// 更新传感器输入
void Motion_Get_sensorData(T_motion_sensorData* mSensor)
{
	// sensor data from imu
	float x = rot_vec[0];
	float y = rot_vec[1];
	Motion_Norm_2D(&x,&y);
	mSensor->dir_x = x;
	mSensor->dir_y = y;
	
	x = pos_ned_m[0];
	y = pos_ned_m[1];
	mSensor->pos_x = x;
	mSensor->pos_y = y;
	
	
	
	//battery state
	//mSensor->power = power;
	
	
	
	//magnetic sensor data
	int32_t left 	= leftsensor_data; 
	int32_t right   = rightsensor_data;
	
	// 更新超声波传感器
	//mSensor->sonar_l = g_sonar.left;
	//mSensor->sonar_r = g_sonar.right;
	
	
	// 磁导线阈值判断 如果低于MOTION_WIRE_MISSING 则认为无磁导线
	/*
	if(left < 0)
	{
		if(left > -WIRE_MIN)
		{
			mSensor->side_l = MOTION_WIRE_MISSING;
			mSensor->magValue_l = 0;
		}
		else
		{
			mSensor->side_l = MOTION_WIRE_OUTSIDE;
			mSensor->magValue_l = -left;
		}
	}
	else
	{
		if(left < WIRE_MIN)
		{
			mSensor->side_l = MOTION_WIRE_MISSING;
			mSensor->magValue_l = 0;
		}
		else
		{
			mSensor->side_l = MOTION_WIRE_INSIDE;
			mSensor->magValue_l = left;
		}
	}
	
	if(right <0)
	{
		if(right > -WIRE_MIN)
		{
			mSensor->side_r = MOTION_WIRE_MISSING;
			mSensor->magValue_r = 0;
		}
		else
		{
			mSensor->side_r = MOTION_WIRE_OUTSIDE;
			mSensor->magValue_r = -right;
		}
	}
	else
	{
		if(right < WIRE_MIN)
		{
			mSensor->side_r = MOTION_WIRE_MISSING;
			mSensor->magValue_r = 0;
		}
		else
		{
			mSensor->side_r = MOTION_WIRE_INSIDE;
			mSensor->magValue_r = right;
		}
	}
	*/
}

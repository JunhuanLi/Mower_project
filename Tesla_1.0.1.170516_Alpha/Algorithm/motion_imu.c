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
#include "motion_imu.h"
#include "PI.h"
#include "math.h"
#include "global.h"
#include "usart_driver.h"

/* macros *********************************************************************/
T_bool OrientationTuned = TRUE;
/* funcitons ******************************************************************/
/*
void Tracking_Init(T_motion_tracker* mTracker,uint8_t en, T_motion_tracker_state_type state)
{
	mTracker->enable = en;
	mTracker->state = state;
	mTracker->error = TRACKING_ERROR_NONE;
	//mTracker->sensor.mag_polarity = 1;
	Tracking_Set_Path_Param(mTracker,0,0,0);
	Tracking_Set_Angle_Param(mTracker,0,0,0);
	Tracking_Set_Mag_Tracking_Param(mTracker,0,0,0);
	Tracking_Set_Mag_Gotoline_Param(mTracker,0,0,0);
	Tracking_Start_2D_Angle(mTracker,0,0,0);
}
*/

void Motion_Init_2D_followLine_params(T_motion_tracker* mTracker, float advanc, float kp, float ki, float il )
{
	mTracker->followLine_params.advancing = advanc;	
	mTracker->followLine_params.direction_pi.kp             = kp;
	mTracker->followLine_params.direction_pi.ki             = ki;
	mTracker->followLine_params.direction_pi.il 						=	il;
	
	mTracker->followLine_params.position_pi.integral				= 0;
	mTracker->followLine_params.direction_pi.integral				= 0;
}

/*void Motion_Init_2D_Angle(T_motion_tracker* mTracker,float kp, float ki, float il)
{
	mTracker->followLine_params.direction_pi.kp             = kp;
	mTracker->followLine_params.direction_pi.ki             = ki;
	mTracker->followLine_params.direction_pi.il 						=	il;
	
	mTracker->followLine_params.direction_pi.integral				= 0;
}
*/

void Motion_Update_2D_Line(T_motion_tracker* mTracker,float targetPoint_x, float targetPoint_y, float dir_x, float dir_y, float vel)
{
	Motion_Norm_2D(&dir_x,&dir_y);
	mTracker->trackingType  														= MOTION_TRACKING_2D_LINE;
	mTracker->followLine_params.targetPoint_x 										= targetPoint_x;
	mTracker->followLine_params.targetPoint_y 										= targetPoint_y;
	mTracker->followLine_params.dir_x 											= dir_x;
	mTracker->followLine_params.dir_y 											= dir_y;
	mTracker->followLine_params.newDir_x										= dir_x;
	mTracker->followLine_params.newDir_y										= dir_y;
	//mTracker->command_vel													= vel;
	mTracker->target_vel 													= vel;
}

void Motion_Update_2D_Angle(T_motion_tracker* mTracker,float dir_x,float dir_y,float vel)
{
	Motion_Norm_2D(&dir_x,&dir_y);
	mTracker->trackingType															= MOTION_TRACKING_2D_ANGLE;
	mTracker->followLine_params.dir_x 											= dir_x;
	mTracker->followLine_params.dir_y 											= dir_y;	
	mTracker->followLine_params.newDir_x										= dir_x;
	mTracker->followLine_params.newDir_y										= dir_y;
	//mTracker->command_vel 													= vel;
	mTracker->target_vel														= vel;

}

/*
void Motion_Start_2D_Arc(T_motion_tracker* mTracker,float targetPoint_x, float targetPoint_y, float center_x, float center_y,float vel)
{
	mTracker->trackingType 														= MOTION_TRACKING_2D_ARC;
	mTracker->followLine_params.targetPoint_x 										= targetPoint_x;
	mTracker->followLine_params.targetPoint_y 										= targetPoint_y;
	mTracker->followLine_params.center_x 										= center_x;
	mTracker->followLine_params.center_y	 									= center_y;
	mTracker->command_vel													= vel;
	mTracker->target_vel 													= vel;
	mTracker->followLine_params.position_pi.integral				= 0;
	mTracker->followLine_params.direction_pi.integral				= 0;
}

void Motion_Start_2D_Point(T_motion_tracker* mTracker,float targetPoint_x, float targetPoint_y,float vel)
{
	mTracker->trackingType 														= MOTION_TRACKING_2D_POINT;
	mTracker->followLine_params.targetPoint_x 										= targetPoint_x;
	mTracker->followLine_params.targetPoint_y 										= targetPoint_y;
	mTracker->command_vel													= vel;
	mTracker->target_vel														= vel;
	mTracker->followLine_params.position_pi.integral				= 0;
	mTracker->followLine_params.direction_pi.integral				= 0;
}


void Tracking_Convert_3D(T_motion_tracker* mTracker)
{
	
}

*/

void Motion_Run_Tracker(T_motion_tracker* mTracker)
{
	if(mTracker->trackingType == MOTION_TRACKING_2D_ANGLE)
	{
		Motion_Run_2D_Angle(mTracker);
	}
	else if(mTracker->trackingType == MOTION_TRACKING_2D_LINE)
	{
		Motion_Run_2D_Line(mTracker);
	}
}


static float Tracking_2D_Angle(T_motion_tracker* mTracker)
{
	T_motion_turn_type 										rot_dir;
	float																	err;
	volatile float												cross_product;
	volatile float												dot_product;
	float																	pi_out;
	float																	bodyVel_angular;
	
	/************************The Angle Part*****************************/
	// 通过点乘和叉乘 确定实际方向和期望方向的夹角
	cross_product = (mTracker->sensorData.dir_x*mTracker->followLine_params.newDir_y)-(mTracker->followLine_params.newDir_x*mTracker->sensorData.dir_y);
	dot_product = (mTracker->sensorData.dir_x*mTracker->followLine_params.newDir_x)+(mTracker->sensorData.dir_y*mTracker->followLine_params.newDir_y);
	
	/************ Anticlockwise positive; Clockwise negative ********************/
	// 判断修正误差需要的旋转方向 顺时针或者逆时针
	rot_dir = (cross_product < 0) ? MOTION_TURN_CLOCKWISE:MOTION_TURN_COUNTERCLOCKWISE;
	//err_angle = acosf(dot_product/sqrtf(mTracker->robot.dir_x*mTracker->robot.dir_x+mTracker->robot.dir_y*mTracker->robot.dir_y)/
	//															sqrtf(mTracker->followLine_params.dir_x*mTracker->followLine_params.dir_x+mTracker->followLine_params.dir_y*mTracker->followLine_params.dir_y));

	//防止 acosf 返回NaN
	if(dot_product > 1.0f)		
		dot_product = 1.0f;

	//计算误差角度
	err = acosf(dot_product);  //acosf 0 ~ pi (?)
	
	//误差角度取正
	if(rot_dir == MOTION_TURN_COUNTERCLOCKWISE)
		err = -err;
	
	//运行PI控制器
	if(err != 0.0f)
	{	
		pi_out = PI_Run(&(mTracker->followLine_params.direction_pi),0,err);
	}
	else
	{
		rot_dir = MOTION_TURN_NONE;
		pi_out = 0;
	}
	
	bodyVel_angular = pi_out;
	
	return bodyVel_angular;
}

static void Tracking_2D_Line(T_motion_tracker* mTracker)
{
	float																		err_x,err_y;
	float																		adv_x,adv_y;
	float																		dir_x,dir_y;
	float																		distance;
	
	/***************************The Line Part*************************************/
	// 计算机器人到参考直线的距离
	distance = (mTracker->followLine_params.targetPoint_x-mTracker->sensorData.pos_x)
						*(-mTracker->followLine_params.dir_y)
						+(mTracker->followLine_params.targetPoint_y-mTracker->sensorData.pos_y)
						*(mTracker->followLine_params.dir_x);

	// 误差向量的计算
	err_x = -distance * mTracker->followLine_params.dir_y;
	err_y = distance * mTracker->followLine_params.dir_x;
	// 前瞻点向量的计算
	adv_x = mTracker->followLine_params.advancing * mTracker->followLine_params.dir_x;
	adv_y = mTracker->followLine_params.advancing * mTracker->followLine_params.dir_y;
	
	// 更新方向向量的计算
	dir_x = err_x + adv_x;
	dir_y = err_y + adv_y;
		
	// 单位化
	Motion_Norm_2D(&dir_x,&dir_y);
		
	mTracker->followLine_params.newDir_x = dir_x;
	mTracker->followLine_params.newDir_y = dir_y;
}

// 2D 角度控制器
void Motion_Run_2D_Angle(T_motion_tracker* mTracker)
{
	//Tracking_Get_Position_2D(mTracker);
	mTracker->bodyVel_angular = Tracking_2D_Angle(mTracker);
	mTracker->bodyVel_linear = mTracker->target_vel;
}

// 2D 循线控制器
void Motion_Run_2D_Line(T_motion_tracker* mTracker)
{
	//Tracking_Get_Position_2D(mTracker);
	Tracking_2D_Line(mTracker);
	mTracker->bodyVel_angular = Tracking_2D_Angle(mTracker);
	mTracker->bodyVel_linear = mTracker->target_vel;
}

void RotateToNorth(T_motion_tracker* mTracker){
	float eul_angle = eul_rad[0];
	if( fabsf(eul_angle) > 0.05f * 90 / (2* 3.1415926) ){ 
		Motion_Update_2D_Angle(mTracker,1.0f, 0.0f, 1000.0f);
		Motion_Run_Tracker(mTracker);

	} else {
		OrientationTuned = TRUE;
		//Motion_Init_2D_Angle(mTracker, 1000, 0.15, 500.0f);
		Motion_Update_Zigzag(mTracker, 1.0f, 0.0f);
	}
	
}


float Tracking_Vect2Angle(float x, float y)
{
	float pi = 3.14159265;
	volatile float angle;
	if(x != 0)
	{
		angle = atan2f(y,x)/pi*180.0;
	}
	else
	{
		angle = 0;
	}
	return angle;
}


/*******************************************************************************

  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name 	: pi.h
  Author		: George 	
  Version		: V1.0.0	  
  Date			: 2017/07/21
  Description	: pi 
  
  History:		  
				  
	1. Date 		:
	   Author		:
	   Modification :
	2. ...
	
*******************************************************************************/

/* includes *******************************************************************/
#include "motion_control.h"
#include "motor_control.h"
#include "motion_math.h"

/* macros *********************************************************************/

//global
Vector2D direction[4] ;
T_bool flag = FALSE;

/* static variables ***********************************************************/
void Init_Vector2D()
{
	direction[0].x = 1.0f;
	direction[0].y = 0.0f;
	
	direction[1].x = 0.0f;
	direction[1].y = -1.0f;
	
	direction[2].x = -1.0f;
	direction[2].y = 0.0f;
	
	direction[3].x = 0.0f;
	direction[3].y = 1.0f;
}

// 求单位向量
static __inline void Motion_Norm_2D(float* x, float* y)
{
	float length = sqrtf((*x)*(*x) + (*y)*(*y));
	if(length != 0)
	{
		*x = *x / length;
		*y = *y / length;
	}
}

// 工字型路径状态机
static void Motion_Run_Zigzag(T_motion* motion)
{
	if(motion->zigzag.state == T_MOTION_ZIGZAG_STATE_IDLE)
	{
		rt_kprintf("Waiting orders! \n");
		return;
	}
	//直线部分
	else if(motion->zigzag.state == T_MOTION_ZIGZAG_STATE_LINE)
	{
		//运行位姿控制器
		Motion_Update_2D_Line(&motion->tracker, 1.0f, 0.0f, direction[0].x, direction[0].y, 1000.0f);
		Motion_Run_Tracker(&motion->tracker);
		
		//判断是否碰撞、是否跨线
		if(motion->tracker.sensorData.bump_l || motion->tracker.sensorData.bump_r 
																		|| (motion->tracker.sensorData.side_l == MOTION_MAG_LINE_OUTSIDE 
																				&& motion->tracker.sensorData.side_r == MOTION_MAG_LINE_OUTSIDE) )
		{
			flag = !flag;
			motion->zigzag.state 						= T_MOTION_ZIGZAG_STATE_TURN;
			Motion_Update_Zigzag(motion, motion->tracker.sensorData.dir_x, motion->tracker.sensorData.dir_y);
		}
		
	}
	//掉头部分
	else if(motion->zigzag.state == T_MOTION_ZIGZAG_STATE_TURN)
	{
		//float dot_product = (motion->tracker.sensorData.dir_x*motion->zigzag.heading_x)+(motion->tracker.sensorData.dir_y*motion->zigzag.heading_y);
		float dot_product = (motion->tracker.sensorData.dir_x*motion->zigzag.pre_heading_x)+(motion->tracker.sensorData.dir_y*motion->zigzag.pre_heading_y);
		float k = (1- motion->zigzag.blade_bodywidth_ratio * motion->zigzag.blade_overlaping_ratio) / 2;
		
		//计算线速度和角速度
		if(flag) //COUNTERCLOCKWISE
		{
			Motion_Update_2D_Angle(&motion->tracker, direction[1].x, direction[1].y, motion->zigzag.target_vel);
			Motion_Run_Tracker(&motion->tracker);
			motion->tracker.bodyVel_angular /= (1-k);
			//motion->tracker.bodyVel_linear = 0;
		}
		else //CLOCKWISE
		{
			Motion_Update_2D_Angle(&motion->tracker, direction[3].x, direction[3].y, motion->zigzag.target_vel);
			Motion_Run_Tracker(&motion->tracker);
			motion->tracker.bodyVel_angular = - motion->tracker.bodyVel_angular/(1-k);
			//motion->tracker.bodyVel_linear = 0;
		}
		
		//利用点乘判断是否完成掉头 之后计算下一阶段参数

		if( fabs(dot_product) > 0.96f)  //~170degree
			{
				motion->zigzag.state = T_MOTION_ZIGZAG_STATE_LINE;
				motion->tracker.bodyVel_angular = 0;
				motion->tracker.bodyVel_linear = 0;
			}
	
	}
	else if(motion->zigzag.state == T_MOTION_ZIGZAG_STATE_EXCEPTION)
	{
		
	}
}
/* funcitons ******************************************************************/

void Motion_Init(T_motion* motion,uint8_t en)
{
	set_motor_control_type(MOTOR_CONTROL_TYPE_SPEED);
	
	motion->motion_state = 											MOTION_STATE_IDLE;
	motion->exception = 												MOTION_EXCEPTION_NONE;
	motion->enable = 														en;
	
	motion->tracker.trackingType = 							MOTION_TRACKING_2D_ANGLE;
	motion->tracker.acc = 											MOTION_ACC;
	motion->tracker.wheelVel_left = 						0;
	motion->tracker.wheelVel_right	= 					0;
	
	//mTracker->error = TRACKING_ERROR_NONE;
	//mTracker->sensor.mag_polarity = 1;
	Motion_Init_2D_followLine_params(&motion->tracker, 0.4, 2000.0f, 0.1f, 499.9f);
	//Motion_Init_2D_Angle(&motion->tracker,2000,0.1f,500);
	//Motion_Set_Mag_Tracking_Param(&motion->tracker,0,0,0);
	//Motion_Set_Mag_Gotoline_Param(&motion->tracker,0,0,0);
	
	Motion_Init_Zigzag(motion,0.8,0.6, 500.0f);
	
	Init_Vector2D();
}

void Motion_Run(T_motion* motion)
{
	if(1)//zigzag event triggered
	{
		//
		
		
		Motion_Update_Zigzag(motion, motion->tracker.sensorData.dir_x, motion->tracker.sensorData.dir_y);
		Motion_Run_Zigzag(motion);
	}
	

}

/*
void Motion_Process_Obstacle(T_motion* motion)
{
	//Run Obsticle
	float vel_sonar = motion->tracker.command_vel;
	float vel_mag_line = motion->tracker.command_vel;
	uint16_t sonar = motion->tracker.sensorData.sonar_l > motion->tracker.sensorData.sonar_r ? motion->tracker.sensorData.sonar_l : motion->tracker.sensorData.sonar_r;
	
	if(sonar > DECELERATION_SONAR_MIN)
	{
		vel_sonar = (float)(1 - (sonar - DECELERATION_SONAR_MIN))/(float)(DECELERATION_SONAR_MAX - DECELERATION_SONAR_MIN) * (float)(1 -DECELERATION_MIN_SPEED)
		           + (float)DECELERATION_MIN_SPEED;
		vel_sonar = vel_sonar * motion->tracker.command_vel;
	}
	
	motion->tracker.target_vel = vel_sonar < vel_mag_line ? vel_sonar : vel_mag_line;
}
*/

void Motion_Process_Motor_Speed(T_motion* motion)
{
	int32_t vl = (int32_t)(motion->tracker.bodyVel_linear+motion->tracker.bodyVel_angular);
	int32_t vr = (int32_t)(motion->tracker.bodyVel_linear-motion->tracker.bodyVel_angular);
	
	//Run Accellation
	if(motion->tracker.wheelVel_left < vl)
	{
		motion->tracker.wheelVel_left += motion->tracker.acc;
		if(motion->tracker.wheelVel_left > vl)
		{
			motion->tracker.wheelVel_left = vl;
		}
	}
	else if(motion->tracker.wheelVel_left > vl)
	{
		motion->tracker.wheelVel_left -= motion->tracker.acc;
		if(motion->tracker.wheelVel_left < vl)
		{
			motion->tracker.wheelVel_left = vl;
		}
	}
	
	if(motion->tracker.wheelVel_right < vr)
	{
		motion->tracker.wheelVel_right += motion->tracker.acc;
		if(motion->tracker.wheelVel_right > vr)
		{
			motion->tracker.wheelVel_right = vr;
		}
	}
	else if(motion->tracker.wheelVel_right > vr)
	{
		motion->tracker.wheelVel_right -= motion->tracker.acc;
		if(motion->tracker.wheelVel_right < vr)
		{
			motion->tracker.wheelVel_right = vr;
		}
	}
	
	//Set Speed
	set_motor_control_speed_s32(motion->tracker.wheelVel_left,motion->tracker.wheelVel_right);
	//set_motor_control_speed_s32(vl,vr);
}


void Motion_Init_Zigzag(T_motion* motion,float blade_bodywidth_ratio,float blade_overlaping_ratio, float speed)
{
	motion->zigzag.blade_bodywidth_ratio =   		blade_bodywidth_ratio;
	motion->zigzag.blade_overlaping_ratio =  		blade_overlaping_ratio;
	motion->zigzag.state = 									    T_MOTION_ZIGZAG_STATE_IDLE;
	motion->zigzag.f_r = 												T_MOTION_ZIGZAG_GO_FOWARD;
	motion->zigzag.state = 											T_MOTION_ZIGZAG_STATE_LINE;
	motion->zigzag.turn_dir = 									T_MOTION_ZIGZAG_TURN_COUNTERCLOCKWISE;
	motion->zigzag.target_vel = 								speed;
}

void Motion_Update_Zigzag(T_motion* motion, float heading_x, float heading_y)
{
	Motion_Norm_2D(&heading_x,&heading_y);
	
	motion->zigzag.pre_heading_x = 									heading_x;
	motion->zigzag.pre_heading_y = 									heading_y;
	//motion->zigzag.f_r = 													T_MOTION_ZIGZAG_GO_FOWARD;
	motion->motion_state = 													MOTION_STATE_ZIGZAG;
	
	//Motion_Update_2D_Angle(&motion->tracker, motion->zigzag.heading_x, motion->zigzag.heading_y, motion->zigzag.target_vel);
}


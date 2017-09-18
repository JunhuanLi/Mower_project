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
#include "motion_mag_line.h"
#include "PI.h"
#include "math.h"
#include "global.h"
#include "usart_driver.h"

/* macros *********************************************************************/


// 判断车头朝向
/*static T_motion_mag_line_bodyside_type Motion_Wire_Get_bodySide(T_motion_tracker* mTracker)
{		
	// 如果有一个传感器没有读数 则离线太远报错
	if(mTracker->sensorData.side_l == MOTION_WIRE_MISSING 
		|| mTracker->sensorData.side_r == MOTION_WIRE_MISSING)
	{
		return MOTION_WIRE_ORI_MISSING;
	}
	
	// 如果两个传感器在线的同一侧
	if(mTracker->sensorData.side_l == mTracker->sensorData.side_r)
	{
		uint32_t val_l = Motion_Abs(mTracker->sensorData.magValue_l);
		uint32_t val_r = Motion_Abs(mTracker->sensorData.magValue_r);
		// 如果车在线内
		if(mTracker->sensorData.side_l == MOTION_WIRE_INSIDE)
		{
			//如果车头与导线垂直
			if(val_l< val_r * (1.0f + WIRE_FACING_MARGIN)
			&& val_r < val_l * (1.0f + WIRE_FACING_MARGIN))
			{
				return MOTION_WIRE_ORI_IN_NORMAL;
			}
			//如果左边传感器读数较大
			else if(val_l > val_r)
			{
				return MOTION_WIRE_ORI_IN_LHRL;
			}
			//如果右边传感器读数较大
			else
			{
				return MOTION_WIRE_ORI_IN_LLRH;
			}
		}
		//如果车在线外
		else
		{
			//如果车头与导线垂直
			if(val_l< val_r * (1.0f + WIRE_FACING_MARGIN)
			&& val_r < val_l * (1.0f + WIRE_FACING_MARGIN))
			{
				return MOTION_WIRE_ORI_OUT_NORMAL;
			}
			//如果左边传感器读数较大
			else if(val_l > val_r)
			{
				return MOTION_WIRE_ORI_OUT_LHRL;
			}
			//如果右边传感器读数较大
			else
			{
				return MOTION_WIRE_ORI_OUT_LLRH;
			}
		}
	}
	//如果两个传感器在线的不同测
	else
	{
		//如果顺着电流方向循线
		if(mTracker->followWire_params.dir == MOTION_WIRE_DIRECT)
		{
			//如果左边传感器在线内 则车身与前进方向对齐
			if(mTracker->sensorData.side_l == MOTION_WIRE_INSIDE)
			{
				return MOTION_WIRE_ORI_ALIGNED;
			}
			//否则车身相反
			else
			{
				return MOTION_WIRE_ORI_REVERSE;
			}
		}
		//如果逆着电流方向循线
		else
		{
			//如果左边传感器在线外 则车身相反
			if(mTracker->sensorData.side_l == MOTION_WIRE_INSIDE)
			{
				return MOTION_WIRE_ORI_REVERSE;
			}
			//否则车身方向正确
			else
			{
				return MOTION_WIRE_ORI_ALIGNED;
			}
		}
	}
	return MOTION_WIRE_ORI_MISSING;
}

T_motion_mag_line_bodyside_type 			bodyside = 0;

static float Motion_Wire(T_motion_tracker* mTracker)
{
	float 																	left_distance, right_distance;
	static 	T_motion_turn_type 							rot_dir;
	static	T_motion_wire_wheelSide			prev_side;				
	float 																	bodyVel_angular = 0;

	bodyside = Motion_Wire_Get_bodySide(mTracker);	
		
	//如果找不到线报错停车
	if(bodyside == MOTION_WIRE_ORI_MISSING)
	{
		mTracker->error = MOTION_ERROR_NO_WIRE;
		mTracker->bodyVel_angular = 0;
		mTracker->bodyVel_linear = 0;
		return 0;
	}
	
	if(mTracker->followWire_params.state == MOTION_WIRE_STATE_IDLE)
	{
		mTracker->bodyVel_angular = 0;
		mTracker->bodyVel_linear = 0;
		return 0;
	}
	// 初始状态
	else if(mTracker->followWire_params.state == MOTION_WIRE_STATE_START)
	{
		//如果车在线内 则往线的方向走
		if(bodyside == MOTION_WIRE_ORI_IN_NORMAL || bodyside == MOTION_WIRE_ORI_IN_LHRL || bodyside == MOTION_WIRE_ORI_IN_LLRH)
		{
			prev_side = mTracker->sensorData.side_l;
			mTracker->followWire_params.state = MOTION_WIRE_STATE_GOTOLINE; //Maybe in the wrong direction which should be checked later
		}
		//如果车在线外并且与线不垂直则调整车头方向与线垂直
		else if(bodyside == MOTION_WIRE_ORI_OUT_LHRL 
					|| bodyside == MOTION_WIRE_ORI_OUT_LHRL)
		{
			mTracker->followWire_params.state = MOTION_WIRE_STATE_FINDDIR;
		}
		//如果车在线外且与线垂直则往线的方向走（这里没有处理车头方向相反的情况）
		else if(bodyside == MOTION_WIRE_ORI_OUT_NORMAL)
		{
			// may need to turn around
			prev_side = mTracker->sensorData.side_l;
			mTracker->followWire_params.state = MOTION_WIRE_STATE_GOTOLINE;
		}
		//如果已经在线上且车头方向正确则直接开始循线
		else if(bodyside == MOTION_WIRE_ORI_ALIGNED)
		{
			mTracker->followWire_params.state = MOTION_WIRE_STATE_TRACELINE;
		}
		//如果在线上但车头方向不对则原地掉头
		else if(bodyside == MOTION_WIRE_ORI_REVERSE)
		{
			mTracker->followWire_params.state = MOTION_WIRE_STATE_ALIGNING;//May Search for ever, Can be merged with FINDDIR state
			prev_side = MOTION_WIRE_INSIDE;
		}
		mTracker->bodyVel_angular = 0;
		mTracker->bodyVel_linear = 0;
	}
	else if(mTracker->followWire_params.state == MOTION_WIRE_STATE_FINDDIR)
	{
		//左边传感器数大就逆时针转
		if(bodyside == MOTION_WIRE_ORI_OUT_LHRL)
		{
			rot_dir = MOTION_TURN_COUNTERCLOCKWISE;
			mTracker->bodyVel_angular = mTracker->target_vel;
			mTracker->bodyVel_linear = 0;
		}
		//右边传感器数大就顺时针转
		else if(bodyside == MOTION_WIRE_ORI_OUT_LLRH)
		{
			rot_dir = MOTION_TURN_CLOCKWISE;
			mTracker->bodyVel_angular = -mTracker->target_vel;
			mTracker->bodyVel_linear = 0;
		}
		//否则就跳回初始状态
		else
		{
			mTracker->followWire_params.state = MOTION_WIRE_STATE_START;
			mTracker->bodyVel_angular = 0;
			mTracker->bodyVel_linear = 0;
		}
	}
	else if(mTracker->followWire_params.state == MOTION_WIRE_STATE_GOTOLINE)
	{
		//如果检测到跨线则判断车头方向 确定是开始循线 还是先调整车头方向
		if(prev_side != mTracker->sensorData.side_l || prev_side != mTracker->sensorData.side_r)
		{
			//mTracker->followWire_params.state = MOTION_WIRE_STATE_IDLE;
			if(bodyside == MOTION_WIRE_ORI_ALIGNED)
			{
				mTracker->followWire_params.state = MOTION_WIRE_STATE_TRACELINE;
			}
			else
			{
				mTracker->followWire_params.state = MOTION_WIRE_STATE_ALIGNING;
			}
			mTracker->bodyVel_angular = 0;
			mTracker->bodyVel_linear = 0;
		}
		//否则根据传感器数值调整车头方向
		else
		{
			if(mTracker->sensorData.magValue_l > mTracker->sensorData.magValue_r)
			{
				mTracker->bodyVel_angular = 0.25f * mTracker->target_vel;
			}
			else if(mTracker->sensorData.magValue_l < mTracker->sensorData.magValue_r)
			{
				mTracker->bodyVel_angular = -0.25f * mTracker->target_vel;
			}
			else
			{
				mTracker->bodyVel_angular = 0.0f;
			}
			mTracker->bodyVel_linear = mTracker->target_vel;
		}	
	}
	else if(mTracker->followWire_params.state == MOTION_WIRE_STATE_ALIGNING)
	{	
		//调整完成开始循线
		if(bodyside == MOTION_WIRE_ORI_ALIGNED)
		{
			mTracker->followWire_params.state = MOTION_WIRE_STATE_TRACELINE;
			mTracker->bodyVel_angular = 0;
			mTracker->bodyVel_linear = 0;
		}
		//根据循线方向做不同判断
		else if(mTracker->followWire_params.dir == MOTION_WIRE_DIRECT)
		{
			//左传感器在线外就逆时针转 否则顺时针转
			if(mTracker->sensorData.side_l == MOTION_WIRE_OUTSIDE)
			{
				rot_dir = MOTION_TURN_COUNTERCLOCKWISE;
				mTracker->bodyVel_angular = mTracker->target_vel;
				mTracker->bodyVel_linear = 0.25f * mTracker->target_vel;
			}
			else
			{
				rot_dir = MOTION_TURN_CLOCKWISE;
				mTracker->bodyVel_angular = -mTracker->target_vel;
				mTracker->bodyVel_linear = 0.25f * mTracker->target_vel;
			}
		}
		else if(mTracker->followWire_params.dir == MOTION_WIRE_REVERSE)
		{
			if(mTracker->sensorData.side_l == MOTION_WIRE_INSIDE)
			{
				rot_dir = MOTION_TURN_COUNTERCLOCKWISE;
				mTracker->bodyVel_angular = mTracker->target_vel;
				mTracker->bodyVel_linear = 0.25f * mTracker->target_vel;
			}
			else
			{
				rot_dir = MOTION_TURN_CLOCKWISE;
				mTracker->bodyVel_angular = -mTracker->target_vel;
				mTracker->bodyVel_linear = 0.25f * mTracker->target_vel;
			}
		}
		//跳到这里就是有问题了
		else
		{
			while(1);
			mTracker->followWire_params.state = MOTION_WIRE_STATE_IDLE;
			mTracker->bodyVel_angular = 0;
			mTracker->bodyVel_linear = 0;
		}
	}
	else if(mTracker->followWire_params.state == MOTION_WIRE_STATE_TRACELINE)
	{
		float												err;
		float												pi_out;
		
		//如果两个传感器都在线外或者都在线内则大幅度转向
		if(mTracker->sensorData.side_l == mTracker->sensorData.side_r)
		{
			if(mTracker->followWire_params.dir == MOTION_WIRE_DIRECT)
			{
				if(mTracker->sensorData.side_l == MOTION_WIRE_INSIDE)
				{
					mTracker->bodyVel_angular = -0.75*mTracker->target_vel;
					mTracker->bodyVel_linear = mTracker->target_vel;
				}
				else if(mTracker->sensorData.side_l == MOTION_WIRE_OUTSIDE)
				{
					mTracker->bodyVel_angular = 0.75*mTracker->target_vel;
					mTracker->bodyVel_linear = mTracker->target_vel;
				}
			}
			else
			{
				if(mTracker->sensorData.side_l == MOTION_WIRE_INSIDE)
				{
					mTracker->bodyVel_angular = 0.75*mTracker->target_vel;
					mTracker->bodyVel_linear = mTracker->target_vel;					
				}
				else if(mTracker->sensorData.side_l == MOTION_WIRE_OUTSIDE)
				{
					mTracker->bodyVel_angular = -0.75*mTracker->target_vel;
					mTracker->bodyVel_linear = mTracker->target_vel;
				}
			}
		}
		//否则小幅度调整或者不调整
		else
		{
			mTracker->bodyVel_angular = 0;
			mTracker->bodyVel_linear = mTracker->target_vel;		
			if(mTracker->sensorData.side_l != mTracker->sensorData.side_r)
			{
				if(mTracker->sensorData.magValue_l < 15000)
				{
					mTracker->bodyVel_angular = 0.25f*mTracker->target_vel;
					mTracker->bodyVel_linear = mTracker->target_vel;		
				}
				else if(mTracker->sensorData.magValue_r < 15000)
				{
					mTracker->bodyVel_angular = -0.25f*mTracker->target_vel;
					mTracker->bodyVel_linear = mTracker->target_vel;	
				}
				else if(mTracker->sensorData.magValue_l > 26000 && mTracker->sensorData.magValue_r > 26000)
				{
					if(mTracker->sensorData.magValue_l < mTracker->sensorData.magValue_r)
					{
						mTracker->bodyVel_angular = -0.1f*mTracker->target_vel;
						mTracker->bodyVel_linear = mTracker->target_vel;	
					}
					else
					{
						mTracker->bodyVel_angular = -0.1f*mTracker->target_vel;
						mTracker->bodyVel_linear = mTracker->target_vel;	
					}
				}
				else
				{
					mTracker->bodyVel_angular = 0.0f;
					mTracker->bodyVel_linear = mTracker->target_vel;
				}
			}
		}

		/*
		else
		{
			left_distance = sqrtf(WIRE_MAX - mTracker->sensorData.magValue_l);
			right_distance = sqrtf(WIRE_MAX - mTracker->sensorData.magValue_r);
			if(left_distance > right_distance)
			{
				rot_dir = MOTION_TURN_CLOCKWISE;
				err = left_distance - right_distance;
			}
			else if(left_distance < right_distance)
			{
				rot_dir = MOTION_TURN_COUNTERCLOCKWISE;
				err = right_distance - left_distance;
			}
			else
			{
				rot_dir = MOTION_TURN_NONE;
				err = 0;
			}
			
			if(err != 0)
				pi_out = PI_Run(&(mTracker->followWire_params.mag_tracking_pi),0,err);
			
			if(rot_dir == MOTION_TURN_CLOCKWISE)
			{
				//bodyVel_angular = -pi_out;
				bodyVel_angular = -mTracker->target_vel * 0.2f;
			}
			else if(rot_dir == MOTION_TURN_COUNTERCLOCKWISE)
			{
				//bodyVel_angular = pi_out;
				bodyVel_angular = mTracker->target_vel * 0.2f;
			}
			else
			{
				bodyVel_angular = 0;
			}
			
			if(bodyVel_angular > mTracker->target_vel)
			{
				bodyVel_angular = mTracker->target_vel;
			}
			else if(bodyVel_angular < -mTracker->target_vel)
			{
				bodyVel_angular = -mTracker->target_vel;
			}
			
			mTracker->bodyVel_angular = bodyVel_angular;
			mTracker->bodyVel_linear = mTracker->target_vel;
		}
		*/
/*	}
	else if(mTracker->followWire_params.state == MOTION_WIRE_STATE_DONE)		
	{
		mTracker->followWire_params.state = MOTION_WIRE_STATE_IDLE;
		mTracker->bodyVel_angular = 0;
		mTracker->bodyVel_linear = 0;
	}
	return bodyVel_angular;
}

*/

static T_motion_bodySide Motion_Wire_Get_bodySide(T_motion_tracker* mTracker)
{		
	// 如果有一个传感器没有读数 则离线太远报错
	if(mTracker->sensorData.side_l == MOTION_WIRE_WHEEL_MISSING 
		|| mTracker->sensorData.side_r == MOTION_WIRE_WHEEL_MISSING)
	{
		return MOTION_WIRE_BODY_MISSING;
	}
	
	// 如果两个传感器在线的同一侧
	if(mTracker->sensorData.side_l == mTracker->sensorData.side_r)
	{
		return mTracker->sensorData.side_l == MOTION_WIRE_WHEEL_INSIDE ? MOTION_WIRE_BODY_INSIDE : MOTION_WIRE_BODY_OUTSIDE;
	}
	else
	{
		//如果顺着电流方向循线
		if(mTracker->followWire_params.dir == MOTION_WIRE_DIRECT)
		{
			//如果左边传感器在线内 则车身与前进方向对齐
			return mTracker->sensorData.side_l == MOTION_WIRE_WHEEL_INSIDE ? MOTION_WIRE_BODY_ALIGNED : MOTION_WIRE_BODY_REVERSED;
		}
		//如果逆着电流方向循线
		else
		{
			//如果左边传感器在线外 则车身相反
			return mTracker->sensorData.side_l == MOTION_WIRE_WHEEL_INSIDE ? MOTION_WIRE_BODY_REVERSED : MOTION_WIRE_BODY_ALIGNED;
		}
	}
	return MOTION_WIRE_BODY_MISSING;
}

void Motion_Run_Wire(T_motion_tracker* mTracker)
{
	mTracker->followWire_params.bodyside = Motion_Wire_Get_bodySide(mTracker);	
		
	//如果找不到线报错停车
	if(mTracker->followWire_params.bodyside == MOTION_WIRE_BODY_MISSING)
	{
		mTracker->error = MOTION_ERROR_NO_WIRE;
		mTracker->bodyVel_angular = 0;
		mTracker->bodyVel_linear = 0;
	}
	
	if(mTracker->followWire_params.state == MOTION_WIRE_STATE_IDLE)
	{
		mTracker->bodyVel_angular = 0;
		mTracker->bodyVel_linear = 0;
	}
	// 初始状态
	else if(mTracker->followWire_params.state == MOTION_WIRE_STATE_START)
	{
		switch(mTracker->followWire_params.bodyside){
			case MOTION_WIRE_BODY_INSIDE || MOTION_WIRE_BODY_OUTSIDE:
					mTracker->followWire_params.state = MOTION_WIRE_STATE_FINDDIR; break;
			case MOTION_WIRE_BODY_ALIGNED:
					mTracker->followWire_params.state = MOTION_WIRE_STATE_TRACELINE; break;
			case MOTION_WIRE_BODY_REVERSED: 
				mTracker->followWire_params.state = MOTION_WIRE_STATE_ALIGNING; break;
			default : break;
		}
		mTracker->followWire_params.pre_wheelside = mTracker->sensorData.side_l;
		mTracker->followWire_params.pre_magValue = mTracker->sensorData.magValue_l;
	}
	else if(mTracker->followWire_params.state == MOTION_WIRE_STATE_FINDDIR)
	{
		float magDiff = 0;
		float cOutput = 0;
		
		magDiff = fabsf(mTracker->sensorData.magValue_l-mTracker->sensorData.magValue_r);
		
		
		if( magDiff > 100 ){//?
			cOutput = PI_Run2(&mTracker->followWire_params.mag_gotoline_pi, -magDiff);
			mTracker->bodyVel_angular = -magDiff/5000 * mTracker->target_vel; //?
			mTracker->bodyVel_linear = 0;
		}	
		else
			{
				mTracker->followWire_params.state = MOTION_WIRE_STATE_GOTOLINE;
			}
	}
	else if(mTracker->followWire_params.state == MOTION_WIRE_STATE_GOTOLINE)
	{
		if(mTracker->followWire_params.pre_wheelside == mTracker->sensorData.side_l)
		{
			if(mTracker->followWire_params.pre_magValue <  mTracker->sensorData.magValue_l) 
			{
				//turn around
				//写一个函数
				mTracker->followWire_params.pre_magValue = 100000; //有效防止非线性
			}
			else 
			{
				mTracker->bodyVel_linear = mTracker->target_vel;
				mTracker->bodyVel_angular = 0;
			}
		}
		else
		{
			mTracker->followWire_params.state = MOTION_WIRE_STATE_ALIGNING;
		}
	}

	else if(mTracker->followWire_params.state == MOTION_WIRE_STATE_ALIGNING)
	{	
		//调整完成开始循线
		if(mTracker->followWire_params.bodyside == MOTION_WIRE_BODY_ALIGNED)
		{
			mTracker->followWire_params.state = MOTION_WIRE_STATE_TRACELINE;
			mTracker->bodyVel_angular = 0;
			mTracker->bodyVel_linear = 0;
		}
		//根据循线方向做不同判断
		else 
		{
			mTracker->bodyVel_linear = 0;
			mTracker->bodyVel_angular =  mTracker->target_vel;
		}
	}
		/*
	else if(mTracker->followWire_params.state == MOTION_WIRE_STATE_TRACELINE)
	{
		float												err;
		float												pi_out;
		
		//如果两个传感器都在线外或者都在线内则大幅度转向
		if(mTracker->sensorData.side_l == mTracker->sensorData.side_r)
		{
			if(mTracker->followWire_params.dir == MOTION_WIRE_DIRECT)
			{
				if(mTracker->sensorData.side_l == MOTION_WIRE_INSIDE)
				{
					mTracker->bodyVel_angular = -0.75*mTracker->target_vel;
					mTracker->bodyVel_linear = mTracker->target_vel;
				}
				else if(mTracker->sensorData.side_l == MOTION_WIRE_OUTSIDE)
				{
					mTracker->bodyVel_angular = 0.75*mTracker->target_vel;
					mTracker->bodyVel_linear = mTracker->target_vel;
				}
			}
			else
			{
				if(mTracker->sensorData.side_l == MOTION_WIRE_INSIDE)
				{
					mTracker->bodyVel_angular = 0.75*mTracker->target_vel;
					mTracker->bodyVel_linear = mTracker->target_vel;					
				}
				else if(mTracker->sensorData.side_l == MOTION_WIRE_OUTSIDE)
				{
					mTracker->bodyVel_angular = -0.75*mTracker->target_vel;
					mTracker->bodyVel_linear = mTracker->target_vel;
				}
			}
		}
		//否则小幅度调整或者不调整
		else
		{
			mTracker->bodyVel_angular = 0;
			mTracker->bodyVel_linear = mTracker->target_vel;		
			if(mTracker->sensorData.side_l != mTracker->sensorData.side_r)
			{
				if(mTracker->sensorData.magValue_l < 15000)
				{
					mTracker->bodyVel_angular = 0.25f*mTracker->target_vel;
					mTracker->bodyVel_linear = mTracker->target_vel;		
				}
				else if(mTracker->sensorData.magValue_r < 15000)
				{
					mTracker->bodyVel_angular = -0.25f*mTracker->target_vel;
					mTracker->bodyVel_linear = mTracker->target_vel;	
				}
				else if(mTracker->sensorData.magValue_l > 26000 && mTracker->sensorData.magValue_r > 26000)
				{
					if(mTracker->sensorData.magValue_l < mTracker->sensorData.magValue_r)
					{
						mTracker->bodyVel_angular = -0.1f*mTracker->target_vel;
						mTracker->bodyVel_linear = mTracker->target_vel;	
					}
					else
					{
						mTracker->bodyVel_angular = -0.1f*mTracker->target_vel;
						mTracker->bodyVel_linear = mTracker->target_vel;	
					}
				}
				else
				{
					mTracker->bodyVel_angular = 0.0f;
					mTracker->bodyVel_linear = mTracker->target_vel;
				}
			}
		}
	}
	else if(mTracker->followWire_params.state == MOTION_WIRE_STATE_DONE)		
	{
		mTracker->followWire_params.state = MOTION_WIRE_STATE_IDLE;
		mTracker->bodyVel_angular = 0;
		mTracker->bodyVel_linear = 0;
	}*/
}

void Motion_Set_Wire_Tracking_Param(T_motion_tracker* mTracker,float kp, float ki, float il)
{
	mTracker->followWire_params.mag_tracking_pi.kp = kp;
	mTracker->followWire_params.mag_tracking_pi.ki = ki;
	mTracker->followWire_params.mag_tracking_pi.il = il;
}

void Motion_Set_Wire_Gotoline_Param(T_motion_tracker* mTracker,float kp, float ki, float il)
{
	mTracker->followWire_params.mag_gotoline_pi.kp = kp;
	mTracker->followWire_params.mag_gotoline_pi.ki = ki;
	mTracker->followWire_params.mag_gotoline_pi.il = il;
}

void Motion_Start_Wire(T_motion_tracker* mTracker,float vel,T_motion_wire_dir dir)
{
	mTracker->trackingType															 		= MOTION_TRACKING_WIRE;
	mTracker->followWire_params.state	 											= MOTION_WIRE_STATE_START;
	mTracker->command_vel																		= vel;
	mTracker->target_vel																		= vel;
	mTracker->followWire_params.dir	 												= dir;
	mTracker->followWire_params.mag_gotoline_pi.integral		= 0;
	mTracker->followWire_params.mag_tracking_pi.integral		= 0;
}



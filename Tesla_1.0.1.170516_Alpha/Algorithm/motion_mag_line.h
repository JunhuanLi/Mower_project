/*******************************************************************************

  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name 	: path_tracking.h
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
#ifndef __MOTION_WIRE__
#define __MOTION_WIRE__

#include "stm32f4xx.h"
#include "typedef.h"
#include "pi.h"
#include "motion_math.h"
#include "motion_types.h"

void Motion_Run_Wire(T_motion_tracker* mTracker);

void Motion_Set_Wire_Tracking_Param(T_motion_tracker* mTracker,float kp, float ki, float il);
void Motion_Set_Wire_Gotoline_Param(T_motion_tracker* mTracker,float kp, float ki, float il);

void Motion_Start_Wire(T_motion_tracker* mTracker,float vel,T_motion_wire_dir dir);

#endif

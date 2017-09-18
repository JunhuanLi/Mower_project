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

#ifndef __MOTION_TYPES_H__
#define __MOTION_TYPES_H__

#include "stm32f4xx.h"
#include "typedef.h"
#include "pi.h"

#define WIRE_POLARITY
#define WIRE_MAX						52000.0f
#define WIRE_MIN						50.0f
#define WIRE_FACING_MARGIN				0.2f
#define MOTION_ACC							60  //4500 /s

// ¼õËÙãÐÖµ
#define DECELERATION_SONAR_MAX					
#define DECELERATION_SONAR_MIN
#define DECELERATION_WIRE_MAX
#define DECELERATION_MIN_SPEED				(0.1f)


// Î»×Ë¿ØÖÆÆ÷×´Ì¬Ã¶¾ÙÁ¿
typedef enum
{
	MOTION_TRACKING_2D_ANGLE = 0,
	MOTION_TRACKING_2D_LINE,
	MOTION_TRACKING_2D_ARC,
	MOTION_TRACKING_2D_POINT,
	MOTION_TRACKING_3D_ANGLE,
	MOTION_TRACKING_3D_LINE,
	MOTION_TRACKING_3D_ARC,
	MOTION_TRACKING_3D_POINT,
	MOTION_TRACKING_WIRE
}T_tracking_type;

// ´íÎó×´Ì¬Ã¶¾ÙÁ¿
typedef enum
{
	MOTION_ERROR_NONE = 0,
	MOTION_ERROR_NO_WIRE,				//The Mag Line is not found
}T_motion_err_type;

// ½ÇËÙ¶È·½ÏòÃ¶¾ÙÁ¿
typedef enum
{
	MOTION_TURN_NONE = 0,
	MOTION_TURN_CLOCKWISE,
	MOTION_TURN_COUNTERCLOCKWISE
}T_motion_turn_type;

// ´Åµ¼ÏßÄÚÍâÃ¶¾ÙÁ¿
typedef enum
{
	MOTION_WIRE_WHEEL_MISSING = 0,
	MOTION_WIRE_WHEEL_INSIDE,
	MOTION_WIRE_WHEEL_OUTSIDE
}T_motion_wire_wheelSide;

// ´Åµ¼ÏßÑ­Ïß·½ÏòÃ¶¾ÙÁ¿
typedef enum
{
	MOTION_WIRE_DIRECT = 0,				//counter clockwise
	MOTION_WIRE_REVERSE					//clockwise
}T_motion_wire_dir;

// ´Åµ¼Ïß¿ØÖÆÆ÷×´Ì¬Ã¶¾ÙÁ¿
typedef enum
{
	MOTION_WIRE_STATE_IDLE = 0,
	MOTION_WIRE_STATE_START,		// ÆðÊ¼×´Ì¬
	MOTION_WIRE_STATE_FINDDIR,   	// Ñ°ÕÒ´Åµ¼Ïß·½Ïò Ö±µ½Óë´Åµ¼Ïß´¹Ö±
	MOTION_WIRE_STATE_GOTOLINE,		// Ç°½øÖ±µ½¿ç¹ýÏß
	MOTION_WIRE_STATE_ALIGNING,		// µ÷Õû³µÍ··½ÏòÖ±µ½×óÓÒ´«¸ÐÆ÷ÔÚ³µÍ·µÄÕýÈ··½Ïò
	MOTION_WIRE_STATE_TRACELINE,	// Ñ­Ïß½×¶Î
	MOTION_WIRE_STATE_DONE
}T_motion_wire_state;

//typedef enum
//{
//	MOTION_WIRE_ORI_MISSING = 0,
//	MOTION_WIRE_ORI_IN_LHRL,			//Left High Right Low inside 
//	MOTION_WIRE_ORI_IN_LLRH,			//Left Low Right High inside
//	MOTION_WIRE_ORI_IN_NORMAL,		//´¹Ö±                inside 
//	MOTION_WIRE_ORI_OUT_LHRL,			//Left High Right Low outside
//	MOTION_WIRE_ORI_OUT_LLRH,			//Left Low Right High outside
//	MOTION_WIRE_ORI_OUT_NORMAL,		//´¹Ö±	              outside
//	MOTION_WIRE_ORI_REVERSE, 
//	MOTION_WIRE_ORI_ALIGNED
//}T_motion_mag_line_orientation_type;

typedef enum
{
	MOTION_WIRE_BODY_MISSING = 0,
	MOTION_WIRE_BODY_INSIDE,			 
	MOTION_WIRE_BODY_OUTSIDE,		
	MOTION_WIRE_BODY_REVERSED, 
	MOTION_WIRE_BODY_ALIGNED
}T_motion_bodySide;

typedef struct			//for path tracking all position is in unit of meter 
{
	volatile float 										targetPoint_x;		//used for line, arc and point
	volatile float 										targetPoint_y;		//used for line, arc and point
	volatile float 										targetPoint_z;		//used for line, arc and point
	volatile float 										dir_x;			//used for line
	volatile float 										dir_y;			//used for line
	volatile float 										dir_z;			//used for line
	//float 					center_x;		//used for arc
	//float 					center_y;		//used for arc
	//float 					center_z;		//used for arc
	
	float															advancing;		//Advancing (in meters) for path tracking
	volatile float										newDir_x;		//The adjusted direction(after position control)
	volatile float										newDir_y;
	volatile float 										newDir_z;
	
	T_pi															position_pi;
	T_pi															direction_pi;
}T_motion_followLineParams;

typedef struct
{
	T_motion_wire_dir									dir;								// direct = counter-clockwise reverse = clockwise
	T_motion_wire_state								state;
	T_motion_bodySide									bodyside;
	T_motion_wire_wheelSide						pre_wheelside;
	float                             pre_magValue;
	T_pi															mag_gotoline_pi;
	T_pi															mag_tracking_pi;
}T_motion_followWireParams;

typedef struct
{
	//Sensed robot position
	float pos_x;
	float pos_y;
	float pos_z;
	
	//Sensed robot orientation
	float dir_x;
	float dir_y;
	float dir_z;
	
	//Sensed robot velocity
	float vel;
	
	//The voltage of the battery
	float power; //Current voltage
	float fullPower; //Voltage when the battery is full
	
	//Mag line sensor
	T_motion_wire_wheelSide						side_l;							//1: inside    -1:outside   0:too far
	T_motion_wire_wheelSide						side_r;
	float 														magValue_l;						// the larger the closer
	float 														magValue_r;
	
	//Ultrasonic
	float 														sonar_l;
	float 														sonar_r;
	
	//Bump
	uint8_t 													bump_l;
	uint8_t 													bump_r;
	
	//Lift
	uint8_t														lift_l;
	uint8_t														lift_r;
}T_motion_sensorData;

typedef struct
{
	T_motion_err_type									error;
	T_tracking_type 									trackingType;
	T_motion_followLineParams					followLine_params;
	T_motion_followWireParams					followWire_params;
	T_motion_sensorData								sensorData;		
	
	float															acc;							//Acceleration (a parameter) for the wheel
	float															wheelVel_left;						//The current velocity for wheel l
	float															wheelVel_right;						//The current velocity for wheel r
	float															command_vel;			//Velocity Command
	float															target_vel;				//target velocity 
	float															bodyVel_linear;					//The velocity(linear) considering acceleration and obstacle
	float															bodyVel_angular;
}T_motion_tracker;


#endif

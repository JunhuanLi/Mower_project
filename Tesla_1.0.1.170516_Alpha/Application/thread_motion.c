
/*************************************************
  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name:		thread_motion.c
  Author:			Raymond
  Date:				2017.7.7
  Version:        
  Description:    // 用于详细说明此程序文件完成的主要功能，与其他模块
                  // 或函数的接口，输出值、取值范围、含义及参数间的控
                  // 制、顺序、独立或依赖等关系
  History:        // 修改历史记录列表，每条修改记录应包括修改日期、修改
                  // 者及修改内容简述  
    1. Date:
       Author:
       Modification:
    2. ...
*************************************************/


#include "stm32f4xx.h"
#include <rtthread.h>
//#include <string.h>
#include <stdio.h>
#include "mower_common.h"
#include "motion_control.h"
#include "global.h"
#include "motor_control.h"
#include "usart_driver.h"
#include "lcd12864_io_spi.h"

/*********TEST FUNCTIONS******************
static void mower_motion_square(T_motion* motion,float speed, uint32_t side_length)
{
	uint8_t state = 0;
	uint32_t count = 0;
	
	rt_uint32_t recved;
	
	Motion_Update_2D_Angle(&motion->tracker,1.0f,0.0f,speed);
	while(1)
	{
		rt_event_recv(&sys_event, SYS_EVN_MOTION, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);

		Motion_Get_imuData(&motion->tracker.sensorData);
		Motion_Get_sensorData(&motion->tracker.sensorData);
		
		if(state == 0 || state == 2 || state == 4 || state == 6)
		{
			if(count >= side_length)
			{
				state ++;
				count = 0;
			}
			count++;
			Motion_Run_Tracker(&motion->tracker);
		}
		else if(state == 1)
		{
			motion->tracker.bodyVel_linear = 0;
			motion->tracker.bodyVel_angular = speed;
			
			if(motion->tracker.sensorData.dir_y < -0.96f)
			{
				Motion_Update_2D_Angle(&motion->tracker,0.0f,-1.0f,speed);
				state ++;
			}
		}
		else if(state == 3)
		{
			motion->tracker.bodyVel_linear = 0;
			motion->tracker.bodyVel_angular = speed;
			
			if(motion->tracker.sensorData.dir_x < -0.96f)
			{
				Motion_Update_2D_Angle(&motion->tracker,-1.0f,0.0f,speed);
				state ++;
			}
		}
		else if(state == 5)
		{
			motion->tracker.bodyVel_linear = 0;
			motion->tracker.bodyVel_angular = speed;
			
			if(motion->tracker.sensorData.dir_y > 0.96f)
			{
				Motion_Update_2D_Angle(&motion->tracker,0.0f,1.0f,speed);
				state ++;
			}
		}
		else if(state == 7)
		{
			motion->tracker.bodyVel_linear = 0;
			motion->tracker.bodyVel_angular = speed;
			
			if(motion->tracker.sensorData.dir_x > 0.96f)
			{
				Motion_Update_2D_Angle(&motion->tracker,1.0f,0.0f,speed);
				state = 0;
			}
		}
		Motion_Process_Motor_Speed(motion);
		update_motor_control();
	}
}
*/
const float DIR_POS_OFFSET = 0.96f;
//const float DIR_NEG_OFFSET = -DIR_POS_OFFSET;
const float DIR_NEG_OFFSET = 0.007f;
	
static void mower_motion_square_position(T_motion* motion,float speed, float side_length)
{
	uint8_t state = 0;
	uint32_t count = 0;
	float point_x = 0;
	float point_y = 0;
	
	rt_uint32_t recved;
	
	point_x = side_length;
	point_y = 0.0f;
	Motion_Start_2D_Line(&motion->tracker,point_x,point_y,1.0f,0.0f,speed);
	while(1)
	{
		rt_event_recv(&sys_event, SYS_EVN_MOTION, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);

		Motion_Get_imuData(&motion->tracker.sensorData);
		Motion_Get_sensorData(&motion->tracker.sensorData);
		
		if(state == 0 || state == 2 || state == 4 || state == 6)
		{
			float dist2 = (point_x - motion->tracker.sensorData.pos_x) * (point_x - motion->tracker.sensorData.pos_x) + (point_y - motion->tracker.sensorData.pos_y) * (point_y - motion->tracker.sensorData.pos_y);
			if(dist2 < 0.0025f)  //dist < 0.05m = 5cm
				state ++;

			Motion_Run_Tracker(&motion->tracker);
		}
		else if(state == 1)
		{
			Motion_Start_2D_Angle(&motion->tracker, 0.0f, -1.0f, speed);
			Motion_Run_Tracker(&motion->tracker);
			motion->tracker.bodyVel_linear = 0;

			if( fabsf(motion->tracker.sensorData.dir_y + 1.0f) < DIR_NEG_OFFSET)
			{
				point_x = side_length;
				point_y = -side_length;
				Motion_Start_2D_Line(&motion->tracker,point_x,point_y,0.0f,-1.0f,speed);	
				state ++;
			}
		}
		else if(state == 3)
		{
			//motion->tracker.bodyVel_linear = 0;
			//motion->tracker.bodyVel_angular = speed;
			Motion_Start_2D_Angle(&motion->tracker, -1.0f, 0.0f,speed);
			Motion_Run_Tracker(&motion->tracker);
			motion->tracker.bodyVel_linear = 0;

			if(fabsf(motion->tracker.sensorData.dir_x + 1.0f) < DIR_NEG_OFFSET)
			{
				point_x = 0.0f;
				point_y = -side_length;
				Motion_Start_2D_Line(&motion->tracker,point_x,point_y,-1.0f,0.0f,speed);
				state ++;
			}
		}
		else if(state == 5)
		{
			//motion->tracker.bodyVel_linear = 0;
			//motion->tracker.bodyVel_angular = speed;
			Motion_Start_2D_Angle(&motion->tracker, 0.0f, 1.0f, speed);
			Motion_Run_Tracker(&motion->tracker);
			motion->tracker.bodyVel_linear = 0;

			if(fabsf(motion->tracker.sensorData.dir_y - 1.0f) < DIR_NEG_OFFSET)
			{
				point_x = 0.0f;
				point_y = 0.0f;
				Motion_Start_2D_Line(&motion->tracker,point_x,point_y,0.0f,1.0f,speed);
				state ++;
			}
		}
		else if(state == 7)
		{
			//motion->tracker.bodyVel_linear = 0;
			//motion->tracker.bodyVel_angular = speed;
			Motion_Start_2D_Angle(&motion->tracker, 1.0f, 0.0f, speed);
			Motion_Run_Tracker(&motion->tracker);
			motion->tracker.bodyVel_linear = 0;

			if(fabsf(motion->tracker.sensorData.dir_x - 1.0f) < DIR_NEG_OFFSET)
			{
				point_x = side_length;
				point_y = 0.0f;
				Motion_Start_2D_Line(&motion->tracker,point_x,point_y,1.0f,0.0f,speed);
				state = 0;
			}
		}
		Motion_Process_Motor_Speed(motion);
		update_motor_control();
	}
}

/*static void mower_motion_circle(T_motion* motion,float line_speed, float angle_speed)
{
	float current_angle = 0;
	float tx = 1.0f;
	float ty = 0.0f;
	rt_uint32_t recved;
	uint8_t buf[100] = "hello!!!!!!!!!\n\r";
	while(1)
	{
		volatile float sin = sinf(current_angle);
		volatile float cos = cosf(current_angle);
		volatile float x = tx;
		volatile float y = ty;
		volatile float tx = x*cos-y*sin;
		volatile float ty = x*sin+y*cos;
		
		rt_event_recv(&sys_event, SYS_EVN_MOTION, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);
		
		//rt_debug(buf,16);
		Motion_Get_imuData(&motion->tracker.sensorData);
		Motion_Get_sensorData(&motion->tracker.sensorData);
		Motion_Update_2D_Angle(&motion->tracker,tx,ty,line_speed);
		Motion_Run_Tracker(&motion->tracker);
		Motion_Process_Motor_Speed(motion);
		
		current_angle -= angle_speed;
		if(current_angle < 0)
		{
			current_angle += 2*3.1415926;
		}
		
		update_motor_control();
	}
}
*/
/********END OF TEST FUNCTIONS************/






/*在分配堆栈空间时，必须要对其*/

/*static void mower_lineTest(T_motion* motion,float speed, float line_length)
{
	uint8_t state = 0;
	uint32_t count = 0;
	
	rt_uint32_t recved;
	
	while(1)
	{
		rt_event_recv(&sys_event, SYS_EVN_MOTION, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);

		Motion_Get_imuData(&motion->tracker.sensorData);
		Motion_Get_sensorData(&motion->tracker.sensorData);
		
		if(state == 0)
		{
			Motion_Update_2D_Line(&motion->tracker,1.0f, 0.0f, 1.0f, 0.0f, speed); 
			Motion_Run_Tracker(&motion->tracker);
			
			float dist2 = (1.0f - motion->tracker.sensorData.pos_x)*(1.0f - motion->tracker.sensorData.pos_x)
											+ (0.0f - motion->tracker.sensorData.pos_y)*(0.0f - motion->tracker.sensorData.pos_y);
			if(dist2 < 0.0025f)  //dist < 0.05m = 5cm
				state+=1;
		}
		else if(state == 1)
		{
			Motion_Update_2D_Angle(&motion->tracker, 0.0f, -1.0f, speed);
			Motion_Run_Tracker(&motion->tracker);
			motion->tracker.bodyVel_linear = 0;

			if( fabsf(motion->tracker.sensorData.dir_y + 1.0f) <0.05f)
			{
				state = 0;
			}
		}
		
		Motion_Process_Motor_Speed(motion);
		update_motor_control();
		//rt_kprintf("\r\n%d,%d",(int)(motion->tracker.bodyVel_linear*1000),(int)motion->tracker.bodyVel_angular*1000);
	}
	
}
*/
ALIGN(RT_ALIGN_SIZE)
char thread_motion_stack[1024];
struct rt_thread thread_motion;

void mower_motion_thread(void* parameter)
{
  rt_uint32_t recved;
	T_motion motion;
	
	//rt_thread_delay(2000); //need be removed later
	
	Motion_Init(&motion,1);
	
  //while(is_odo_ready==0);
	//mower_motion_square_position(&motion,600,1.2f);
	//mower_motion_square(&motion,400,700);//10 sec
	//mower_motion_circle(&motion,300,0.0031416);//40 sec
	

	//Motion_Start_Wire(&motion.tracker,500,MOTION_WIRE_DIRECT);
	//Motion_Update_Zigzag(&motion, 1.0f, 0.0f);
	
	while (1)
	{
		rt_event_recv(&sys_event, SYS_EVN_MOTION, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);
		//rt_enter_critical();	//	LCD_PWM = 0;
		
		//Update all the sensors first
		//Motion_Get_imuData(&motion.tracker.sensorData);
		Motion_Get_sensorData(&motion.tracker.sensorData);
		//Motion_Process_Obstacle(&motion);
		
		
		
		//call updates
		//Motion_Update_2D_Angle(&motion.tracker,1.0f,0.0f,500);
		//Motion_Update_2D_Line(&motion.tracker,0.0f,0.0f,1.0f,0.0f,500);
		//Motion_Update_Zigzag(&motion, 1.0f, 0.0f,);
		
		
		
		
		//Run controller
		//Motion_Run_Wire(&motion.tracker);
		//Motion_Run_Tracker(&motion.tracker);
		//Motion_Run(&motion);
		//Update Motor Command
		//motion.tracker.bodyVel_angular = 0;
		//motion.tracker.bodyVel_linear = 1000.0f;

		//Debug
		//rt_kprintf("angle = %d                x = %d                 y = %d \n\r",(int)(eul_rad[0]*10*57.3), (int)(motion.tracker.sensorData.pos_x*100), (int)(motion.tracker.sensorData.pos_y*100));
		//rt_kprintf("left = %d, right =%d \n\r",(int)(motion.tracker.bodyVel_linear),(int)(motion.tracker.bodyVel_angular));
		//rt_kprintf("left = %d, right = %d \r\n\r",leftsensor_data,rightsensor_data);
		
		Motion_Process_Motor_Speed(&motion);
		update_motor_control();
		//rt_event_send(&sys_event,SYS_EVN_MOTOR);
			//	LCD_PWM = 1;
			//	rt_exit_critical();
		}
 
}









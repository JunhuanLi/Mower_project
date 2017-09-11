
/*************************************************
  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name:		thread_motor_control.c
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
#include <stdio.h>
#include "mower_common.h"
#include "motor_control.h"
ALIGN(RT_ALIGN_SIZE)
char thread_motor_stack[1024];
struct rt_thread thread_motor;

void mower_motor_thread(void* parameter)
{
    rt_uint32_t recved;
	
    while(1)
    {
        rt_event_recv(&sys_event, SYS_EVN_MOTOR, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);
//		motor_test();
		update_motor_control();
		#if 0//def CUSTOMER_SERIAL_DEBUG
		
		sprintf(str_buffer, "motor %d", mpu_idt++);
        rt_kprintf("\r\n...mower_motor_thread %s  ",str_buffer);
        #endif
		//rt_thread_delay(20);
    }
}


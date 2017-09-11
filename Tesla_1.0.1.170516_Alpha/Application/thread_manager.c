
/*************************************************
  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name:			thread_manager.c
  Author:				Raymond
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


ALIGN(RT_ALIGN_SIZE)
char thread_manager_stack[1024];
struct rt_thread thread_manager;

void mower_manager_thread(void* parameter)
{
    //char str_buffer[256];
    //int mpu_idt = 0;
    rt_uint32_t recved;
    
    while(1)
    {
        rt_event_recv(&sys_event, SYS_EVN_MANAGER, RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);
		#if 0//def CUSTOMER_SERIAL_DEBUG
		// 将第三个参数，转换成第二个参数格式，把最终结果放到第一个参数里
		// 需要头文件<stdio.h>
		sprintf(str_buffer, "manager %d", mpu_idt++);
        rt_kprintf("\r\n...__manager_thread %s  ",str_buffer);
        #endif
		//rt_thread_delay(20);// 20ms
    }
}


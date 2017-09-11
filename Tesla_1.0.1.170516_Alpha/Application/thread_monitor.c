
/*************************************************
  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name:			thread_monitor.c
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
//#include <string.h>
#include <stdio.h>
#include "mower_common.h"



/* allocate space */

ALIGN(RT_ALIGN_SIZE)
char thread_monitor_stack[1024];
struct rt_thread thread_monitor;
void mower_monitor_thread(void* parameter)
{
	//rt_tick_t cnt;
	//int gps_idx = 0;
	//char xstr_buffer[256];
	rt_uint32_t receive_event;
    while (1)
    {

		#if 0//def CUSTOMER_SERIAL_DEBUG
		/*
		C99标准库函数，功能是将第三个参数，转换成第二个参数的格式，然后放到第一个参数里
		需要头文件<stdio.h>
		*/
        sprintf(xstr_buffer, "monitor %d", gps_idx++); 
        rt_kprintf("\r\n...__monitor_thread %s  ",xstr_buffer);
		#endif

        /* real time */
        if(sys_power_state) // 开机状态下
        {
            rt_event_recv(&exception_event,
                EVENT_EXC_LOWER_POWER|EVENT_EXC_OUT_OF_AREA\
                |EVENT_EXC_NO_SINGNAL|EVENT_EXC_LIFT\
                |EVENT_EXC_TRAPPED|EVENT_EXC_TOO_STEEP\
                |EVENT_EXC_BM_OVERCURRENT|EVENT_EXC_BM_OVERLOAD\
                |EVENT_EXC_WM_OVERCURRENT|EVENT_EXC_WM_OVERLOAD,
                RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,
                RT_WAITING_FOREVER,
                &receive_event);
            
           #ifdef CUSTOMER_SERIAL_DEBUG
           rt_kprintf("\r\n...monitor:%s...",receive_event);
		   #endif
           
            if(receive_event & EVENT_EXC_LOWER_POWER)
            {
                
            }
            if(receive_event & EVENT_EXC_OUT_OF_AREA)
            {
                
            }
            if(receive_event & EVENT_EXC_NO_SINGNAL)
            {

            }
            if(receive_event & EVENT_EXC_LIFT)
            {
                
            }
            if(receive_event & EVENT_EXC_TRAPPED)
            {
                
            }
            if(receive_event & EVENT_EXC_TOO_STEEP)
            {
                
            }
            if(receive_event & EVENT_EXC_BM_OVERCURRENT)
            {
                
            }
            if(receive_event & EVENT_EXC_BM_OVERLOAD)
            {
                
            }
            if(receive_event & EVENT_EXC_WM_OVERCURRENT)
            {
                
            }
            if(receive_event & EVENT_EXC_WM_OVERLOAD)
            {
                
            }


            
        }
    }
}









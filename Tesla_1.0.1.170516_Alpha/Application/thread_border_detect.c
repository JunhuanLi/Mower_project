/*************************************************
  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name:		thread_border_detect.c
  Author:			Raymond
  Date:				2017.8.8
  Version:        
  Description:    //
                  // 
                  // 
  History:        // 修改历史记录列表，每条修改记录应包括修改日期、修改
                  // 者及修改内容简述  
    1. Date:
       Author:
       Modification:
    2. ...
*************************************************/


#include <rtthread.h>
#include "stm32f4xx.h"
#include "mower_common.h"
#include "stdlib.h"

ALIGN(RT_ALIGN_SIZE)
char thread_border_stack[512];
struct rt_thread thread_border;

void mower_border_detect_thread(void* parameter)
{
	rt_uint32_t recved;
    //u8 * p = RT_NULL;
    //u8 * pp = RT_NULL;
	while(1)
	{
        rt_event_recv(&sys_event,SYS_EVN_BORDER, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);
		#if 0//def CUSTOMER_SERIAL_DEBUG
		rt_kprintf("\r\n...border>>>");
		
        p = malloc(8);
        pp = rt_malloc(8);

        *p = 1;
        *(p+1) = 2;
        *(pp+7) = 10;

        rt_kprintf("\r\n...p::%d  %d >>>",*p,*(p+1));
        rt_kprintf("\r\n...pp::%d >>>",*(pp+7));
        
		free(p);
        rt_free(pp);
        #endif
            
	}
}




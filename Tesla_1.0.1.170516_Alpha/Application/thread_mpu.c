
/*************************************************
  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name:			thread_mpu.c
  Author:				Raymond
  Date:				2017.5.16
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


ALIGN(RT_ALIGN_SIZE)
char thread_mpu_stack[1024];
struct rt_thread thread_mpu;

void mower_mpu_thread(void* parameter)
{

    char str_buffer[256];
		int mpu_idt = 0;
	
    while(1)
    {
		// 将第三个参数，转换成第二个参数格式，把最终结果放到第一个参数里
		// 需要头文件<stdio.h>
		#if 0//def CUSTOMER_SERIAL_DEBUG
		sprintf(str_buffer, "mpu %d", mpu_idt++);
        rt_kprintf("\r\n...mower_mpu_thread %s  ",str_buffer);
        #endif
		//rt_thread_delay(RT_TICK_PER_SECOND);
    }
}


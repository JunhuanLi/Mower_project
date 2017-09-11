
/*************************************************
  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name:		thread_data_tx.c
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



/*在分配堆栈空间时，必须要对其*/
ALIGN(RT_ALIGN_SIZE)
char thread_datarx_stack[1024];
struct rt_thread thread_datarx;
void mower_data_rx_thread(void* parameter)
{
	rt_tick_t cnt;
	int gps_idx = 0;
	char xstr_buffer[256];
    while (1)
    {
		/*
		C99标准库函数，功能是将第三个参数，转换成第二个参数的格式，然后放到第一个参数里
		需要头文件<stdio.h>
		*/
		#if 0//def CUSTOMER_SERIAL_DEBUG
        sprintf(xstr_buffer, "data_rx %d", gps_idx++); 
        rt_kprintf("\r\n...__data_rx_thread %s  ",xstr_buffer);
		#endif
        //rt_thread_delay(20);// 20ms
    }
}









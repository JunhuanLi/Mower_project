/*************************************************
  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name:			thread_rainbow_led.c
  Author:				Raymond
  Date:				2017.6.9
  Version:        
  Description:    // 七彩灯控制相关线程 在application.c中调用mower_rainbowled_thread()函数
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
#include "rainbow_led.h"



ALIGN(RT_ALIGN_SIZE)
char thread_rainbowled_stack[512];
struct rt_thread thread_rainbowled;

void mower_rainbowled_thread(void* parameter)
{
	static u8 i;
	while(1)
	{
		#if 0//def CUSTOMER_SERIAL_DEBUG
		rt_kprintf("\r\nthis is mower_rainbowled_thread ..");
		#endif
		
		
		/* 挂起10*10=100ms */
		rt_thread_delay(50);
		i++;
		i=i%7;

		switch(i)
		{
			case 0:
				LED_R_ON;
				LED_G_OFF;
				LED_B_OFF;
				break;
				
			case 1:
				LED_R_OFF;
				LED_G_ON;
				LED_B_OFF;
				break;
			case 2:
				LED_R_OFF;
				LED_G_OFF;
				LED_B_ON;
				break;
			case 3:
				LED_R_ON;
				LED_G_ON;
				LED_B_OFF;
				break;
			case 4:
				LED_R_ON;
				LED_G_OFF;
				LED_B_ON;
				break;
			case 5:
				LED_R_OFF;
				LED_G_ON;
				LED_B_ON;
				break;
			case 6:
				LED_R_ON;
				LED_G_ON;
				LED_B_ON;
				break;
			default:
				break;
		}

		
		
	}
}


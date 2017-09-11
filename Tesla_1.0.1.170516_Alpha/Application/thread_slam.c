/*************************************************
  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name:		thread_slam.c
  Author:			Raymond
  Date:				2017.7.7
  Version:        
  Description:    // �߲ʵƿ�������߳� ��application.c�е���mower_rainbowled_thread()����
                  // 
                  // 
  History:        // �޸���ʷ��¼�б�ÿ���޸ļ�¼Ӧ�����޸����ڡ��޸�
                  // �߼��޸����ݼ���  
    1. Date:
       Author:
       Modification:
    2. ...
*************************************************/


#include <rtthread.h>
#include "stm32f4xx.h"




ALIGN(RT_ALIGN_SIZE)
char thread_slam_stack[512];
struct rt_thread thread_slam;

void mower_slam_thread(void* parameter)
{
	static u8 i;
	while(1)
	{
		#if 0//def CUSTOMER_SERIAL_DEBUG
		rt_kprintf("\r\n...init_priority = %d..current_priority = %d ..",thread_slam.init_priority,thread_slam.current_priority);
		#endif
		
		
		//rt_thread_delay(200); //20ms
	}
}


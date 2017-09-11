
/*************************************************
  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name:		thread_motor_control.c
  Author:			Raymond
  Date:				2017.7.7
  Version:        
  Description:    // ������ϸ˵���˳����ļ���ɵ���Ҫ���ܣ�������ģ��
                  // �����Ľӿڣ����ֵ��ȡֵ��Χ�����弰������Ŀ�
                  // �ơ�˳�򡢶����������ȹ�ϵ
  History:        // �޸���ʷ��¼�б�ÿ���޸ļ�¼Ӧ�����޸����ڡ��޸�
                  // �߼��޸����ݼ���  
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


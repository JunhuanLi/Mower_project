
/*************************************************
  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name:		thread_system_control.c
  Author:			Raymond
  Date:				2017.7.7
  Version:        
  Description:    // ������ϸ˵���˳����ļ���ɵ���Ҫ���ܣ�������ģ��
                  // �����Ľӿڣ����ֵ��ȡֵ��Χ�����弰������Ŀ�
                  // �ơ�˳�򡢶����������ȹ�ϵ
  History:        // �޸���ʷ��¼�б���ÿ���޸ļ�¼Ӧ�����޸����ڡ��޸�
                  // �߼��޸����ݼ���  
    1. Date:
       Author:
       Modification:
    2. ...
*************************************************/

#include "stm32f4xx.h"
#include <rtthread.h>
#include <stdio.h>


ALIGN(RT_ALIGN_SIZE)
char thread_sysctrl_stack[1024];
struct rt_thread thread_sysctrl;

void mower_sysctrl_thread(void* parameter)
{

    char str_buffer[256];
		int mpu_idt = 0;
	
    while(1)
    {
		// ��������������ת���ɵڶ���������ʽ�������ս���ŵ���һ��������
		// ��Ҫͷ�ļ�<stdio.h>
		#if 0//def CUSTOMER_SERIAL_DEBUG
		sprintf(str_buffer, "system_control %d", mpu_idt++);
        rt_kprintf("\r\n...mower_system_control_thread %s  ",str_buffer);
        #endif
		//rt_thread_delay(20);
    }
}

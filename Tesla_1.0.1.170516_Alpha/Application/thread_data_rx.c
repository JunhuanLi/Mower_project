
/*************************************************
  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name:		thread_data_tx.c
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
//#include <string.h>
#include <stdio.h>



/*�ڷ����ջ�ռ�ʱ������Ҫ����*/
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
		C99��׼�⺯���������ǽ�������������ת���ɵڶ��������ĸ�ʽ��Ȼ��ŵ���һ��������
		��Ҫͷ�ļ�<stdio.h>
		*/
		#if 0//def CUSTOMER_SERIAL_DEBUG
        sprintf(xstr_buffer, "data_rx %d", gps_idx++); 
        rt_kprintf("\r\n...__data_rx_thread %s  ",xstr_buffer);
		#endif
        //rt_thread_delay(20);// 20ms
    }
}









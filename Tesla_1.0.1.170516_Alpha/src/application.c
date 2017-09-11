/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include <stdio.h>
#include "stm32f4xx.h"
#include <board.h>
#include <rtthread.h>

#ifdef RT_USING_LWIP
#include <lwip/sys.h>
#include <lwip/api.h>
#include <netif/ethernetif.h>
#include "stm32_eth.h"
#endif

/* 使用12864屏 */
#ifdef SUPPORT_LCD12864_ST7920
#include "lcd12864_parallel.h"
#endif

/* 使用4.3寸tft屏 */
#ifdef SUPPORT_TFT43
#include "lcd.h"
#endif
#include "rainbow_led.h"
#include "lcd12864_io_spi.h"
#include "mower_common.h"
#include "time_control.h"
#include "thread_app.h"
#include "encoder.h"
#include "mpu6500_spi.h"
#include "math.h"

u8 is_imu_valid;
u8 is_odo_valid;

u8 odo_validation(void)
{
	float odo_r,odo_l;
	u8 flag;
	
	while(flag==0)
	{
		odo_l = get_left_encoder();
		odo_r = get_right_encoder();
		if((odo_l!=0)&&(odo_r!=0))
				flag = 1;
		else flag = 0;
	}
	return flag;
}

u8 imu_validation(void)
{
	T_mpu mpu;
	volatile imu_scaled_body imu;
	u8 flag=0;

	while(flag==0)
	{
		get_mpu_info(&mpu); 
		imu.acc_m_s2[0] = (float)mpu.ax/16384*9.8;
		imu.acc_m_s2[1] = (float)mpu.ay/16384*9.8;
		imu.acc_m_s2[2] = (float)mpu.az/16384*9.8;
		imu.gyro_rps[0] = (float)mpu.gx/GYRO_FACTOR*PI/180;
		imu.gyro_rps[1] = (float)mpu.gy/GYRO_FACTOR*PI/180;
		imu.gyro_rps[2] = (float)mpu.gz/GYRO_FACTOR*PI/180;
		if((fabs(imu.acc_m_s2[0])>1e-6)&&(fabs(imu.acc_m_s2[1])>1e-6)&&(fabs(imu.acc_m_s2[2])>1e-6)&&(fabs(imu.gyro_rps[0])>1e-8)&&(fabs(imu.gyro_rps[1])>1e-8))
				flag = 1;
			else flag = 0;
	}
	return flag;

}

void rt_init_thread_entry(void* parameter)
{

/*********** 以下外设初始化代码 ******************/
	#ifdef CUSTOMER_SERIAL_DEBUG
	rt_kprintf("\r\n.....system peripheral device initialization starting ........ )  \r\n \r\n \r\n");
	#endif

/*************************************************/

    rt_event_init(&sys_event,"sys_event",RT_IPC_FLAG_PRIO);
    rt_event_init(&exception_event,"exp_event",RT_IPC_FLAG_PRIO);
    
#ifdef SUPPORT_LCD12864_ST7920
	lcd12864_parallel_config();
	lcd12864_parallel_init();
#endif
/*************************************************/
/* LED */
	rainbow_led_hw_config();
	rainbow_led_variable_init();

/* KEY */
	key_hw_config();
	key_variable_init();
	key_thread_variable_init();
	
  /*imu and odometer valid*/
  is_imu_valid = imu_validation();
	is_odo_valid = odo_validation();


//lcd12864_st7565r_config();
/*************************************************/
	#ifdef CUSTOMER_SERIAL_DEBUG
	rt_kprintf("\r\n.....system peripheral device initialization END ........ )  \r\n \r\n \r\n");
	#endif
}


int rt_application_init()
{
    rt_thread_t init_thread;
	  rt_err_t init_thread_state;

    rt_timer_t timer_result;

	/* 初始化线程的优先级设为最高 保证初始化代码得到运行 */
#if (RT_THREAD_PRIORITY_MAX == 32)
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 8, 20);
#else
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 80, 20);
#endif
    if (init_thread != RT_NULL)
        rt_thread_startup(init_thread);

    /* 10ms */
    rt_timer_init(&timer_10ms,"t10ms",soft_timeout_10ms,RT_NULL,10,RT_TIMER_FLAG_PERIODIC);
    rt_timer_start(&timer_10ms);
    /* 100ms */
    rt_timer_init(&timer_100ms,"t100ms",soft_timeout_100ms,RT_NULL,100,RT_TIMER_FLAG_PERIODIC);
    rt_timer_start(&timer_100ms);
    /* 1s */
    timer_result = rt_timer_create("t1000ms",soft_timeout_1000ms,RT_NULL,1000,RT_TIMER_FLAG_PERIODIC);
    if(timer_result != RT_NULL)
    {
        rt_timer_start(timer_result);
    }

#if 0
	// thread rainbow-led
    init_thread_state=rt_thread_init(&thread_rainbowled,
                   "_led",
                   mower_rainbowled_thread,
                   RT_NULL,
                   &thread_rainbowled_stack[0],
                   sizeof(thread_rainbowled_stack),10,5);
	if(init_thread_state==RT_EOK)
	{
    	rt_thread_startup(&thread_rainbowled);
	}	
#endif 

#if 1
	// thread gps
    init_thread_state=rt_thread_init(&thread_gps,
                   "_gps",
                   mower_gps_thread,
                   RT_NULL,
                   &thread_gps_stack[0],
                   sizeof(thread_gps_stack),10,30);
	if(init_thread_state==RT_EOK)
	{
    	rt_thread_startup(&thread_gps);
	}
#endif

#if 1
	// thread ui
    init_thread_state=rt_thread_init(&thread_ui,
                   "_ui",
                   mower_ui_thread,
                   RT_NULL,
                   &thread_ui_stack[0],
                   sizeof(thread_ui_stack),10,10);
	if(init_thread_state==RT_EOK)
	{
    	rt_thread_startup(&thread_ui);
	}
#endif 

#if 0
	// thread key 
    init_thread_state=rt_thread_init(&thread_key,
                   "_key",
                   mower_key_thread,
                   RT_NULL,
                   &thread_key_stack[0],
                   sizeof(thread_key_stack),10,5);
	if(init_thread_state==RT_EOK)
	{
    	rt_thread_startup(&thread_key);
	}
#endif

#if 1
        // thread imu
        init_thread_state=rt_thread_init(&thread_imu,
                       "_imu",
                       mower_imu_thread,
                       RT_NULL,
                       &thread_imu_stack[0],
                       sizeof(thread_imu_stack),10,30);
        if(init_thread_state==RT_EOK)
        {
            rt_thread_startup(&thread_imu);
        }
#endif

#if 0
        // thread motor control 
        init_thread_state=rt_thread_init(&thread_motor,
                       "_motor",
                       mower_motor_thread,
                       RT_NULL,
                       &thread_motor_stack[0],
                       sizeof(thread_motor_stack),10,5);
        if(init_thread_state==RT_EOK)
        {
            rt_thread_startup(&thread_motor);
        }
#endif

#if 1
        // thread updata 
        init_thread_state=rt_thread_init(&thread_updata,
                       "_updata",
                       mower_updata_thread,
                       RT_NULL,
                       &thread_updata_stack[0],
                       sizeof(thread_updata_stack),10,5);
        if(init_thread_state==RT_EOK)
        {
            rt_thread_startup(&thread_updata);
        }
#endif

#if 1
        // thread monitor 
        init_thread_state=rt_thread_init(&thread_monitor,
                       "_monitor",
                       mower_monitor_thread,
                       RT_NULL,
                       &thread_monitor_stack[0],
                       sizeof(thread_monitor_stack),10,5);
        if(init_thread_state==RT_EOK)
        {
            rt_thread_startup(&thread_monitor);
        }
#endif

#if 1
        // thread motion
        init_thread_state=rt_thread_init(&thread_motion,
                       "_motion",
                       mower_motion_thread,
                       RT_NULL,
                       &thread_motion_stack[0],
                       sizeof(thread_motion_stack),10,5);
        if(init_thread_state==RT_EOK)
        {
            rt_thread_startup(&thread_motion);
        }
#endif

#if 0
        // thread manager 
        init_thread_state=rt_thread_init(&thread_manager,
                       "_manager",
                       mower_manager_thread,
                       RT_NULL,
                       &thread_manager_stack[0],
                       sizeof(thread_manager_stack),10,5);
        if(init_thread_state==RT_EOK)
        {
            rt_thread_startup(&thread_manager);
        }
#endif

#if 1
        // thread wireless 
        init_thread_state=rt_thread_init(&thread_wireless,
                       "_data_rx",
                       mower_wireless_thread,
                       RT_NULL,
                       &thread_wireless_stack[0],
                       sizeof(thread_wireless_stack),10,5);
        if(init_thread_state==RT_EOK)
        {
            rt_thread_startup(&thread_wireless);
        }
#endif

#if 1
        // thread border_detect
        init_thread_state=rt_thread_init(&thread_border,
                       "_border",
                       mower_border_detect_thread,
                       RT_NULL,
                       &thread_border_stack[0],
                       sizeof(thread_border_stack),10,5);
        if(init_thread_state==RT_EOK)
        {
            rt_thread_startup(&thread_border);
        }
#endif

// these threads unused yet
#if 0
#if 1
                // thread data rx 
                init_thread_state=rt_thread_init(&thread_datarx,
                               "_data_rx",
                               mower_data_rx_thread,
                               RT_NULL,
                               &thread_datarx_stack[0],
                               sizeof(thread_datarx_stack),10,5);
                if(init_thread_state==RT_EOK)
                {
                    rt_thread_startup(&thread_datarx);
                }
#endif
        
#if 1
                // thread data tx 
                init_thread_state=rt_thread_init(&thread_datatx,
                               "_data_tx",
                               mower_data_tx_thread,
                               RT_NULL,
                               &thread_datatx_stack[0],
                               sizeof(thread_datatx_stack),10,5);
                if(init_thread_state==RT_EOK)
                {
                    rt_thread_startup(&thread_datatx);
                }
#endif

#if 0
        // thread mpu
        init_thread_state=rt_thread_init(&thread_mpu,
                       "_mpu",
                       mower_mpu_thread,
                       RT_NULL,
                       &thread_mpu_stack[0],
                       sizeof(thread_mpu_stack),10,10);
        if(init_thread_state==RT_EOK)
        {
            rt_thread_startup(&thread_mpu);
        }
#endif


#if 0
        // thread system_control
        init_thread_state=rt_thread_init(&thread_sysctrl,
                       "_system_control",
                       mower_sysctrl_thread,
                       RT_NULL,
                       &thread_sysctrl_stack[0],
                       sizeof(thread_sysctrl_stack),10,5);
        if(init_thread_state==RT_EOK)
        {
            rt_thread_startup(&thread_sysctrl);
        }
#endif
#if 1
        // thread slam 
        init_thread_state=rt_thread_init(&thread_slam,
                       "_slam",
                       mower_slam_thread,
                       RT_NULL,
                       &thread_slam_stack[0],
                       sizeof(thread_slam_stack),10,5);
        if(init_thread_state==RT_EOK)
        {
            rt_thread_startup(&thread_slam);
        }
#endif

#endif

    return 0;
}

/*@}*/

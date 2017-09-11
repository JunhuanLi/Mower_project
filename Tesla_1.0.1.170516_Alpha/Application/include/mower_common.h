/*************************************************
  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name:			mower_common.h
  Author:				Raymond
  Date:				2017.6.7
  Version:        
  Description:		存放公共函数 
  						
  						
  History:        // 修改历史记录列表，每条修改记录应包括修改日期、修改
                  // 者及修改内容简述  
    1. Date:
       Author:
       Modification:
    2. ...
*************************************************/


#ifndef __MOWER_COMMON_H__
#define __MOWER_COMMON_H__

#include "stm32f4xx.h"
#include "rtthread.h"


// sys event 
#define SYS_EVN_KEY_SCAN        (1<<0)  // 启动按键扫描事件
#define SYS_EVN_KEY             (1<<1)  // 产生按键事件
#define SYS_EVN_UI_UPDATA       (1<<2)  // ui更新 200ms一次
#define SYS_EVN_SENSOR_UPDATA   (1<<3)  // 传感器更新事件
#define SYS_EVN_GPS             (1<<4)  // gps线程驱动事件
#define SYS_EVN_IMU             (1<<5)  // imu线程驱动事件
#define SYS_EVN_MOTOR           (1<<6)  // motor线程驱动事件
#define SYS_EVN_WIRELESS        (1<<7)  // wireless线程驱动事件
#define SYS_EVN_BORDER          (1<<8)  // border线程驱动事件
#define SYS_EVN_MOTION          (1<<9)  // motion线程驱动事件 20ms
#define SYS_EVN_MANAGER         (1<<11) // manager线程驱动事件


// exception event
#define EVENT_EXC_LOWER_POWER       (1<<0) 
#define EVENT_EXC_OUT_OF_AREA       (1<<1)
#define EVENT_EXC_NO_SINGNAL        (1<<2)
//#define EVENT_EXC_OVER_CURRENT  (1<<3)
#define EVENT_EXC_LIFT              (1<<4) // lift up
#define EVENT_EXC_TRAPPED           (1<<5) // trap
#define EVENT_EXC_TOO_STEEP         (1<<6)
#define EVENT_EXC_BM_OVERCURRENT    (1<<7)
#define EVENT_EXC_BM_OVERLOAD       (1<<8)
#define EVENT_EXC_WM_OVERCURRENT    (1<<9)
#define EVENT_EXC_WM_OVERLOAD       (1<<10)













typedef enum WORK_TAG
{
 /*01*/ WORK_STA_STANDBY,               // 待机模式
 /*02*/ WORK_STA_BACK_TO_STATION,       // 返回模式
 /*03*/ WORK_STA_RAIN_ESCAPE,           // 避雨模式   
 /*04*/ WORK_STA_ERROR,                 // 故障模式
 /*05*/ WORK_STA_SPOT_CUT,              // 定点模式 
 /*06*/ WORK_STA_MEMERY_CUT,            // 断点模式 
 /*07*/ WORK_STA_AUTOMATIC_CUT,         // 全局模式 
 /*08*/ WORK_STA_IDLE,                  // 空闲模式 
 
 /*09*/ WORK_STA_TOTAL, 
}e_work_state;


// beep enum
typedef enum BEEP_ATTR_TAG
{
 /*01*/ BEEP_ATTR_SINGLE_PASS ,//circulation
 /*02*/ BEEP_ATTR_CIRCUL,
 /*03*/ BEEP_ATTR_CONTINUE,

 /*04*/ BEEP_ATTR_TOTAL,
}e_beep_attr;

struct BEEP_TAG
{   
    e_beep_attr attr;
    u32 period; 
    FunctionalState state;
};
typedef struct BEEP_TAG * e_beep_stru_t;

extern struct BEEP_TAG beep;



#if 0
////////////////////////////////////////////////////////////////////////////////// 
//Ex_NVIC_Config专用定义
#define GPIO_A 				0
#define GPIO_B 				1
#define GPIO_C				2
#define GPIO_D 				3
#define GPIO_E 				4
#define GPIO_F 				5
#define GPIO_G 				6 
#define GPIO_H 				7 
#define GPIO_I 				8 

#define FTIR   				1  		//下降沿触发
#define RTIR   				2  		//上升沿触发

#if 0
//GPIO设置专用宏定义
#define GPIO_MODE_IN    	0		//普通输入模式
#define GPIO_MODE_OUT		1		//普通输出模式
#define GPIO_MODE_AF		2		//AF功能模式
#define GPIO_MODE_AIN		3		//模拟输入模式

#define GPIO_SPEED_2M		0		//GPIO速度2Mhz
#define GPIO_SPEED_25M		1		//GPIO速度25Mhz
#define GPIO_SPEED_50M		2		//GPIO速度50Mhz
#define GPIO_SPEED_100M		3		//GPIO速度100Mhz

#define GPIO_PUPD_NONE		0		//不带上下拉
#define GPIO_PUPD_PU		1		//上拉
#define GPIO_PUPD_PD		2		//下拉
#define GPIO_PUPD_RES		3		//保留 

#define GPIO_OTYPE_PP		0		//推挽输出
#define GPIO_OTYPE_OD		1		//开漏输出 
#endif
//GPIO引脚编号定义
#define PIN0				1<<0
#define PIN1				1<<1
#define PIN2				1<<2
#define PIN3				1<<3
#define PIN4				1<<4
#define PIN5				1<<5
#define PIN6				1<<6
#define PIN7				1<<7
#define PIN8				1<<8
#define PIN9				1<<9
#define PIN10				1<<10
#define PIN11				1<<11
#define PIN12				1<<12
#define PIN13				1<<13
#define PIN14				1<<14
#define PIN15				1<<15 
////////////////////////////////////////////////////////////////////////////////// 
u8 Sys_Clock_Set(u32 plln,u32 pllm,u32 pllp,u32 pllq);		//系统时钟设置
void Stm32_Clock_Init(u32 plln,u32 pllm,u32 pllp,u32 pllq); //时钟初始化  
void Sys_Soft_Reset(void);      							//系统软复位
void Sys_Standby(void);         							//待机模式 	
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset);	//设置偏移地址
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group);			//设置NVIC分组
void MY_NVIC_Init(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group);//设置中断
void Ex_NVIC_Config(u8 GPIOx,u8 BITx,u8 TRIM);				//外部中断配置函数(只对GPIOA~I)
void GPIO_AF_Set(GPIO_TypeDef* GPIOx,u8 BITx,u8 AFx);		//GPIO复用功能设置
void GPIO_Set(GPIO_TypeDef* GPIOx,u32 BITx,u32 MODE,u32 OTYPE,u32 OSPEED,u32 PUPD);//GPIO设置函数  

void sys_power_control(void);


/* 以下汇编函数 */
void WFI_SET(void);		//执行WFI指令
void INTX_DISABLE(void);//关闭所有中断
void INTX_ENABLE(void);	//开启所有中断
void MSR_MSP(u32 addr);	//设置堆栈地址 
#endif

extern struct rt_event sys_event;
extern struct rt_event exception_event;

extern rt_mq_t sys_messagequeue;
extern e_work_state sys_work_state;
extern FunctionalState beep_flag;

extern T_u32_to_bit sys_flag;
#define sys_power_state sys_flag.bitfield.bit0



void beep_ctrl(FunctionalState state,e_beep_attr attr,u32 period);
void beep_moniter(e_beep_stru_t p);


#endif
/* Copyright (C), 2017-2027, TOPBAND Co., Ltd. ********************************/

/*************************************************
  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name:			mower_common.h
  Author:				Raymond
  Date:				2017.6.7
  Version:        
  Description:		��Ź������� 
  						
  						
  History:        // �޸���ʷ��¼�б�ÿ���޸ļ�¼Ӧ�����޸����ڡ��޸�
                  // �߼��޸����ݼ���  
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
#define SYS_EVN_KEY_SCAN        (1<<0)  // ��������ɨ���¼�
#define SYS_EVN_KEY             (1<<1)  // ���������¼�
#define SYS_EVN_UI_UPDATA       (1<<2)  // ui���� 200msһ��
#define SYS_EVN_SENSOR_UPDATA   (1<<3)  // �����������¼�
#define SYS_EVN_GPS             (1<<4)  // gps�߳������¼�
#define SYS_EVN_IMU             (1<<5)  // imu�߳������¼�
#define SYS_EVN_MOTOR           (1<<6)  // motor�߳������¼�
#define SYS_EVN_WIRELESS        (1<<7)  // wireless�߳������¼�
#define SYS_EVN_BORDER          (1<<8)  // border�߳������¼�
#define SYS_EVN_MOTION          (1<<9)  // motion�߳������¼� 20ms
#define SYS_EVN_MANAGER         (1<<11) // manager�߳������¼�


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
 /*01*/ WORK_STA_STANDBY,               // ����ģʽ
 /*02*/ WORK_STA_BACK_TO_STATION,       // ����ģʽ
 /*03*/ WORK_STA_RAIN_ESCAPE,           // ����ģʽ   
 /*04*/ WORK_STA_ERROR,                 // ����ģʽ
 /*05*/ WORK_STA_SPOT_CUT,              // ����ģʽ 
 /*06*/ WORK_STA_MEMERY_CUT,            // �ϵ�ģʽ 
 /*07*/ WORK_STA_AUTOMATIC_CUT,         // ȫ��ģʽ 
 /*08*/ WORK_STA_IDLE,                  // ����ģʽ 
 
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
//Ex_NVIC_Configר�ö���
#define GPIO_A 				0
#define GPIO_B 				1
#define GPIO_C				2
#define GPIO_D 				3
#define GPIO_E 				4
#define GPIO_F 				5
#define GPIO_G 				6 
#define GPIO_H 				7 
#define GPIO_I 				8 

#define FTIR   				1  		//�½��ش���
#define RTIR   				2  		//�����ش���

#if 0
//GPIO����ר�ú궨��
#define GPIO_MODE_IN    	0		//��ͨ����ģʽ
#define GPIO_MODE_OUT		1		//��ͨ���ģʽ
#define GPIO_MODE_AF		2		//AF����ģʽ
#define GPIO_MODE_AIN		3		//ģ������ģʽ

#define GPIO_SPEED_2M		0		//GPIO�ٶ�2Mhz
#define GPIO_SPEED_25M		1		//GPIO�ٶ�25Mhz
#define GPIO_SPEED_50M		2		//GPIO�ٶ�50Mhz
#define GPIO_SPEED_100M		3		//GPIO�ٶ�100Mhz

#define GPIO_PUPD_NONE		0		//����������
#define GPIO_PUPD_PU		1		//����
#define GPIO_PUPD_PD		2		//����
#define GPIO_PUPD_RES		3		//���� 

#define GPIO_OTYPE_PP		0		//�������
#define GPIO_OTYPE_OD		1		//��©��� 
#endif
//GPIO���ű�Ŷ���
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
u8 Sys_Clock_Set(u32 plln,u32 pllm,u32 pllp,u32 pllq);		//ϵͳʱ������
void Stm32_Clock_Init(u32 plln,u32 pllm,u32 pllp,u32 pllq); //ʱ�ӳ�ʼ��  
void Sys_Soft_Reset(void);      							//ϵͳ��λ
void Sys_Standby(void);         							//����ģʽ 	
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset);	//����ƫ�Ƶ�ַ
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group);			//����NVIC����
void MY_NVIC_Init(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group);//�����ж�
void Ex_NVIC_Config(u8 GPIOx,u8 BITx,u8 TRIM);				//�ⲿ�ж����ú���(ֻ��GPIOA~I)
void GPIO_AF_Set(GPIO_TypeDef* GPIOx,u8 BITx,u8 AFx);		//GPIO���ù�������
void GPIO_Set(GPIO_TypeDef* GPIOx,u32 BITx,u32 MODE,u32 OTYPE,u32 OSPEED,u32 PUPD);//GPIO���ú���  

void sys_power_control(void);


/* ���»�ຯ�� */
void WFI_SET(void);		//ִ��WFIָ��
void INTX_DISABLE(void);//�ر������ж�
void INTX_ENABLE(void);	//���������ж�
void MSR_MSP(u32 addr);	//���ö�ջ��ַ 
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

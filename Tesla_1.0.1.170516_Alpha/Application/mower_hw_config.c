/*************************************************
  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name:			mower_hw_config.c
  Author:				Raymond
  Date:				2017.6.13
  Version:        
  Description:		存放一些公共的函数
  						
  						
  History:        // 修改历史记录列表，每条修改记录应包括修改日期、修改
                  // 者及修改内容简述  
    1. Date:
       Author:
       Modification:
    2. ...
*************************************************/
#include "stm32f4xx.h"
#include "board.h" 
#include "usart.h"
#include "mower_hw_config.h"
#include "time_control.h"
#include "hardware.h"


/************************************************
 *
 * 外设硬件配置
 *
 ************************************************/
static void rcc_configuration(void)
{
    rt_uint32_t ahb1_def;
    rt_uint32_t ahb2_def;
    rt_uint32_t ahb3_def;

    rt_uint32_t apb1_def;
    rt_uint32_t apb2_def;

    ahb1_def = ahb2_def = ahb3_def = apb1_def = apb1_def =0;

/******** enable AHB1 clock ************/
    ahb1_def = (AHB1_A | AHB1_B | AHB1_C | AHB1_D\
	         | AHB1_E | AHB1_F | AHB1_G | AHB1_H\
	         | AHB1_I | AHB1_J | AHB1_K);
#ifdef RT_USING_DMA1
    apb1_def |= AHB1_DMA1;
#endif	         
	RCC_AHB1PeriphClockCmd(ahb1_def,ENABLE);



/******** enable AHB2 clock ************/
    ahb2_def = 0;



/******** enable AHB3 clock ************/
#ifdef SUPPORT_TFT43
    ahb3_def = RCC_AHB3Periph_FMC;
#endif
	RCC_AHB3PeriphClockCmd(ahb3_def, ENABLE);



/******** enable APB1 clock ************/
#ifdef RT_USING_UART2
    apb1_def |= RCC_APB1Periph_USART2;
#endif

#ifdef RT_USING_UART3
    apb1_def |= RCC_APB1Periph_USART3;
#endif

//#ifdef RT_USING_UART8
    apb1_def |= RCC_APB1Periph_UART8;
//#endif

    apb1_def |= APB1_TIM6;
    RCC_APB1PeriphClockCmd(apb1_def, ENABLE);


/******** enable APB2 clock ************/
#ifdef RT_USING_UART1
	apb2_def |= RCC_APB2Periph_USART1;
    RCC_APB2PeriphClockCmd(apb2_def, ENABLE);
#endif

}


static void gpio_configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
/*************************************************************
 *         GPIO A CONFIGURATION
 *************************************************************/
// TEST PIN PA0
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

// USART1/2
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

#ifdef RT_USING_UART1
	/* Configure USART1 Rx/tx PIN */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_9;// UART1_GPIO_RX | UART1_GPIO_TX
	GPIO_Init(UART1_GPIO, &GPIO_InitStructure);
    /* Connect alternate function */
    GPIO_PinAFConfig(UART1_GPIO, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(UART1_GPIO, GPIO_PinSource10, GPIO_AF_USART1);
#endif

#ifdef RT_USING_UART2
	/* Configure USART2 Rx/tx PIN */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_Init(UART2_GPIO, &GPIO_InitStructure);
    /* Connect alternate function */
    GPIO_PinAFConfig(UART2_GPIO, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(UART2_GPIO, GPIO_PinSource3, GPIO_AF_USART2);
#endif

//#ifdef RT_USING_UART8
	/* Configure USART2 Rx/tx PIN */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(UART8_GPIO, &GPIO_InitStructure);
    /* Connect alternate function */
    GPIO_PinAFConfig(UART8_GPIO, GPIO_PinSource0, GPIO_AF_UART8);
    GPIO_PinAFConfig(UART8_GPIO, GPIO_PinSource1, GPIO_AF_UART8);
//#endif

/*************************************************************
 *         GPIO B CONFIGURATION
 *************************************************************/
/////LED
#if 1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
#endif

/////USART3
#ifdef RT_USING_UART3
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	/* Configure USART3 Rx/tx PIN */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_Init(UART3_GPIO, &GPIO_InitStructure);
    /* Connect alternate function */
    GPIO_PinAFConfig(UART3_GPIO, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(UART3_GPIO, GPIO_PinSource11, GPIO_AF_USART3);
#endif

#ifdef SUPPORT_TFT43
/////4.3TFT
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
#endif


/*************************************************************
 *         GPIO C CONFIGURATION
 *************************************************************/
#ifdef SUPPORT_LCD12864_ST7920
/////LCD12864 PARALLEL
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
#endif
/*************************************************************
 *         GPIO D CONFIGURATION
 *************************************************************/
 #ifdef SUPPORT_TFT43
 /////4.3TFT FMC
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	//GPIO_PinAFConfig(GPIOD,0,12);	//PD0,AF12
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FMC);
	//GPIO_PinAFConfig(GPIOD,1,12);	//PD1,AF12
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FMC);
	//GPIO_PinAFConfig(GPIOD,4,12);	//PD4,AF12
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FMC);
	//GPIO_PinAFConfig(GPIOD,5,12);	//PD5,AF12 
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FMC);
	//GPIO_PinAFConfig(GPIOD,7,12);	//PD7,AF12
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource7, GPIO_AF_FMC);
	//GPIO_PinAFConfig(GPIOD,8,12);	//PD8,AF12
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FMC);
	//GPIO_PinAFConfig(GPIOD,9,12);	//PD9,AF12
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FMC);
	//GPIO_PinAFConfig(GPIOD,10,12);	//PD10,AF12 
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FMC);
	//GPIO_PinAFConfig(GPIOD,13,12);	//PD13,AF12 
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_FMC);
	//GPIO_PinAFConfig(GPIOD,14,12);	//PD14,AF12
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FMC);
	//GPIO_PinAFConfig(GPIOD,15,12);	//PD15,AF12
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FMC);
#endif

	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
/*************************************************************
 *         GPIO E CONFIGURATION
 *************************************************************/
 #ifdef SUPPORT_TFT43
  /////4.3TFT FMC
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	//GPIO_PinAFConfig(GPIOE,7,12);	//PE7,AF12
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_FMC);
	//GPIO_PinAFConfig(GPIOE,8,12);	//PE8,AF12
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_FMC);
	//GPIO_PinAFConfig(GPIOE,9,12);	//PE9,AF12
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_FMC);
	//GPIO_PinAFConfig(GPIOE,10,12);	//PE10,AF12
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_FMC);
	//GPIO_PinAFConfig(GPIOE,11,12);	//PE11,AF12
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_FMC);
	//GPIO_PinAFConfig(GPIOE,12,12);	//PE12,AF12
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_FMC);
	//GPIO_PinAFConfig(GPIOE,13,12);	//PE13,AF12
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_FMC);
	//GPIO_PinAFConfig(GPIOE,14,12);	//PE14,AF12
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_FMC);
	//GPIO_PinAFConfig(GPIOE,15,12);	//PE15,AF12
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource15, GPIO_AF_FMC);
#endif
/*************************************************************
 *         GPIO F CONFIGURATION
 *************************************************************/


/*************************************************************
 *         GPIO G CONFIGURATION
 *************************************************************/
    ///// KEY COLUMN
       GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
       GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_4;
       GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
       GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
       GPIO_Init(GPIOG,&GPIO_InitStructure);
       PG_OUT(0) = 0;
       PG_OUT(1) = 0;
       PG_OUT(2) = 0;
       PG_OUT(4) = 0;
       /////KEY ROW
       GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
       //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_5 | GPIO_Pin_9 | GPIO_Pin_10;
       GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
       GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
       GPIO_Init(GPIOG,&GPIO_InitStructure);
       PG_OUT(6) = 0;
       PG_OUT(7) = 0;
       PG_OUT(5) = 0;
       PG_OUT(9) = 0;
       PG_OUT(10) = 0;

/*************************************************************
 *         GPIO H CONFIGURATION
 *************************************************************/

/*************************************************************
 *         GPIO I CONFIGURATION
 *************************************************************/
#ifdef SUPPORT_LCD12864_ST7920
/////LCD12864 PARALLEL
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_Init(GPIOI,&GPIO_InitStructure);
#endif
#ifdef SUPPORT_LCD12864_ST7565R
/////LCD12864 st7565R
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_Init(GPIOG,&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_Init(GPIOF,&GPIO_InitStructure);
#endif


/*************************************************************
 *         GPIO J CONFIGURATION
 *************************************************************/

/*************************************************************
 *         GPIO K CONFIGURATION
 *************************************************************/

}

static void nvic_configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

#ifdef RT_USING_UART1
	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif

#ifdef RT_USING_UART2
	/* Enable the USART2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif
	
//	#ifdef RT_USING_UART8
	/* Enable the USART2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = UART8_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
//#endif

#ifdef RT_USING_UART3
	/* Enable the USART3 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    /* Enable the DMA1 Channel2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif

//  TIMER 6
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}


static void dma_configuration(void)
{
#if defined (RT_USING_UART3)
	DMA_InitTypeDef DMA_InitStructure;
    /* Configure DMA Stream */
    DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)0;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = (uint32_t)0;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    DMA_DeInit(UART3_TX_DMA);
    DMA_Init(UART3_TX_DMA, &DMA_InitStructure);
    DMA_ITConfig(UART3_TX_DMA, DMA_IT_TC | DMA_IT_TE, ENABLE);
	//DMA_ClearFlag(DMA1_FLAG_TC5);
#endif
}

static void timer_configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    
    TIM_TimeBaseStructure.TIM_Period = TIME_CONTROL_CLOCK_PERIOD - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = TIME_CONTROL_CLOCK_PRESCALE - 1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
	TIM_TimeBaseInit(TIME_CONTROL_CLOCK_TIMER, &TIM_TimeBaseStructure);

    TIME_CONTROL_CLOCK_TIMER->CNT = 0;
	TIM_ITConfig(TIME_CONTROL_CLOCK_TIMER, TIM_IT_Update, ENABLE);	
	TIM_ClearFlag(TIME_CONTROL_CLOCK_TIMER, TIM_FLAG_Update);
	TIM_Cmd(TIME_CONTROL_CLOCK_TIMER, ENABLE);
}

void peripheral_hw_config(void)
{
	rcc_configuration();
	gpio_configuration();
	nvic_configuration();
	dma_configuration();
    timer_configuration();
    //usart_configuration();
	hardware_init();
}


/************************************************
 *
 * 外设硬件相关硬件初始化
 *
 ************************************************/
void peripheral_hw_init(void)
{
	
}


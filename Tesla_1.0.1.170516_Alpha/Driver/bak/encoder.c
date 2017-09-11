/*******************************************************************************

  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name		: motor.h
  Author		: klaus     
  Version		: V1.0.0      
  Date			: 2017/06/07
  Description	: motor driver 
  
  History:        
                  
	1. Date			:
	   Author		:
	   Modification	:
	2. ...
    
*******************************************************************************/


/* includes *******************************************************************/
#include "encoder.h"

/* macros *********************************************************************/

/* static variables ***********************************************************/
static T_encoder g_encoder = {TRUE, TRUE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* funcitons ******************************************************************/
static void encoder_gpio_configuration(void);
static void encoder_timer_configuration(void);
static void update_left_encoder(void);
static void update_right_encoder(void);

/*******************************************************************************
  Function		: encoder_gpio_configuration      
  Description	: encoder gpio configuration
  Input			: None                    
  Output		: None       
  Return		: None       
*******************************************************************************/
static void encoder_gpio_configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(ENCODER_LEFT_A_CLK | ENCODER_RIGHT_A_CLK, ENABLE);

	GPIO_PinAFConfig(ENCODER_LEFT_A_PORT, ENCODER_LEFT_A_PINSOURCE, GPIO_AF_TIM12);
	GPIO_PinAFConfig(ENCODER_RIGHT_A_PORT, ENCODER_RIGHT_A_PINSOURCE, GPIO_AF_TIM11);

	GPIO_InitStructure.GPIO_Pin = ENCODER_LEFT_A_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ENCODER_LEFT_A_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = ENCODER_RIGHT_A_PIN ;
	GPIO_Init(ENCODER_RIGHT_A_PORT, &GPIO_InitStructure);
}
/*******************************************************************************
  Function		: encoder_timer_configuration      
  Description	: encoder timer configuration
  Input			: None                    
  Output		: None       
  Return		: None       
*******************************************************************************/
static void encoder_timer_configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	RCC_APB1PeriphClockCmd(ENCODER_LEFT_TIM_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(ENCODER_RIGHT_TIM_CLK, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(ENCODER_LEFT_TIMER, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(ENCODER_RIGHT_TIMER, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(ENCODER_LEFT_TIMER, TIM_EncoderMode_TI1,
								TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);

	TIM_EncoderInterfaceConfig(ENCODER_RIGHT_TIMER, TIM_EncoderMode_TI1,
								TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);
								
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 6;
	TIM_ICInit(ENCODER_LEFT_TIMER, &TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_ICFilter = 6;
	TIM_ICInit(ENCODER_RIGHT_TIMER, &TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInit(ENCODER_LEFT_TIMER, &TIM_ICInitStructure);
	TIM_ICInit(ENCODER_RIGHT_TIMER, &TIM_ICInitStructure);

	TIM_ClearFlag(ENCODER_LEFT_TIMER, TIM_FLAG_Update);
	TIM_ITConfig(ENCODER_LEFT_TIMER, TIM_IT_Update, ENABLE);
	TIM_ClearFlag(ENCODER_RIGHT_TIMER, TIM_FLAG_Update);
	TIM_ITConfig(ENCODER_RIGHT_TIMER, TIM_IT_Update, ENABLE);

	ENCODER_LEFT_TIMER->CNT = 27000; 
	ENCODER_RIGHT_TIMER->CNT = 27000; 

	TIM_Cmd(ENCODER_LEFT_TIMER, ENABLE);
	TIM_Cmd(ENCODER_RIGHT_TIMER, ENABLE);
}

/*******************************************************************************
  Function		: encoder_timer_configuration      
  Description	: encoder timer configuration
  Input			: None                    
  Output		: None       
  Return		: None       
*******************************************************************************/
void TIM12_IRQHandler(void)  // is TIM12_IRQHandler exist
{
	TIM_ClearFlag(ENCODER_LEFT_TIMER, TIM_FLAG_Update);
}

/*******************************************************************************
  Function		: encoder_timer_configuration      
  Description	: encoder timer configuration
  Input			: None                    
  Output		: None       
  Return		: None       
*******************************************************************************/
void TIM9_IRQHandler(void) // is TIM9_IRQHandler exist
{
	TIM_ClearFlag(ENCODER_RIGHT_TIMER, TIM_FLAG_Update);
}

/*******************************************************************************
  Function		: encoder_timer_configuration      
  Description	: encoder timer configuration
  Input			: None                    
  Output		: None       
  Return		: None       
*******************************************************************************/
void encoder_init(void)
{
	encoder_gpio_configuration();
	encoder_timer_configuration();
	reset_encoder();
}

/*******************************************************************************
  Function		: update_left_encoder      
  Description	: update left encoder
  Input			: None                    
  Output		: None       
  Return		: None       
*******************************************************************************/
void update_left_encoder(void)
{
	if(g_encoder.left_first_measure_flag == FALSE)
	{
		g_encoder.left_encoder = ENCODER_LEFT_TIMER->CNT;
		g_encoder.left_encoder_delta = g_encoder.left_encoder - g_encoder.last_left_encoder;
		if((ENCODER_LEFT_TIMER->CR1 & TIM_CounterMode_Down) == TIM_CounterMode_Down)
		{
			if(g_encoder.left_encoder < 5535)
			{
				g_encoder.left_encoder_overflow--;
				g_encoder.left_encoder = 60000 + (g_encoder.left_encoder - 5535);
				ENCODER_LEFT_TIMER->CNT = g_encoder.left_encoder;
			}
		}
		else
		{
			if(g_encoder.left_encoder > 60000)
			{
				g_encoder.left_encoder_overflow++;
				g_encoder.left_encoder = 5535 + (g_encoder.left_encoder - 60000);
				ENCODER_LEFT_TIMER->CNT = g_encoder.left_encoder;
			}
		}
			g_encoder.last_left_encoder = g_encoder.left_encoder;
			g_encoder.left_speed = g_encoder.left_encoder_delta * ENCODER_UPDATE_FREQUENCY;
			g_encoder.left_encoder_sum += g_encoder.left_encoder_delta;
	}
	else
	{
		g_encoder.left_first_measure_flag = FALSE;
		g_encoder.left_encoder = 27000;
		g_encoder.last_left_encoder = g_encoder.left_encoder;
		g_encoder.left_encoder_delta = g_encoder.left_encoder - g_encoder.last_left_encoder;
		g_encoder.left_encoder_sum = g_encoder.left_encoder;
		g_encoder.left_encoder_overflow = 0;
		g_encoder.left_speed = 0;
		ENCODER_LEFT_TIMER->CNT = g_encoder.left_encoder;
	}
}

/*******************************************************************************
  Function		: update_right_encoder      
  Description	: update right encoder
  Input			: None                    
  Output		: None       
  Return		: None       
*******************************************************************************/
void update_right_encoder(void)
{
	if(g_encoder.right_first_measure_flag == FALSE)
	{
		g_encoder.right_encoder = ENCODER_RIGHT_TIMER->CNT;
		g_encoder.right_encoder_delta = g_encoder.right_encoder - g_encoder.last_right_encoder;
		if((ENCODER_RIGHT_TIMER->CR1 & TIM_CounterMode_Down) == TIM_CounterMode_Down)
		{
			if(g_encoder.right_encoder < 5535)
			{
				g_encoder.right_encoder_overflow--;
				g_encoder.right_encoder = 60000 + (g_encoder.right_encoder - 5535);
				ENCODER_RIGHT_TIMER->CNT = g_encoder.right_encoder;
			}
		}
		else
		{
			if(g_encoder.right_encoder > 60000)
			{
				g_encoder.right_encoder_overflow++;
				g_encoder.right_encoder = 5535 + (g_encoder.right_encoder - 60000);
				ENCODER_RIGHT_TIMER->CNT = g_encoder.right_encoder;
			}
		}
			g_encoder.last_right_encoder = g_encoder.right_encoder;
			g_encoder.right_speed = g_encoder.right_encoder_delta * ENCODER_UPDATE_FREQUENCY;
			g_encoder.right_encoder_sum+= g_encoder.right_encoder_delta;
	}
	else
	{
		g_encoder.right_first_measure_flag = FALSE;
		g_encoder.right_encoder = 27000;
		g_encoder.last_right_encoder = g_encoder.right_encoder;
		g_encoder.right_encoder_delta = g_encoder.right_encoder - g_encoder.last_right_encoder;
		g_encoder.right_encoder_sum = g_encoder.right_encoder;
		g_encoder.right_encoder_overflow = 0;
		g_encoder.right_speed = 0;
		ENCODER_RIGHT_TIMER->CNT = g_encoder.right_encoder;
	}
}

/*******************************************************************************
  Function		: get_left_motor_speed      
  Description	: get left motor speed
  Input			: None                    
  Output		: None       
  Return		: None       
*******************************************************************************/
s32 get_left_motor_speed(void)
{
	return g_encoder.left_speed;	
}
/*******************************************************************************
  Function		: get_right_motor_speed      
  Description	: get right motor speed
  Input			: None                    
  Output		: None       
  Return		: None       
*******************************************************************************/
s32 get_right_motor_speed(void)
{
	return g_encoder.right_speed;
}
/*******************************************************************************
  Function		: get_left_encoder      
  Description	: get left encoder
  Input			: None                    
  Output		: None       
  Return		: None       
*******************************************************************************/
s64 get_left_encoder(void)
{
	return g_encoder.left_encoder_sum;
}
/*******************************************************************************
  Function		: get_right_encoder      
  Description	: get right encoder
  Input			: None                    
  Output		: None       
  Return		: None       
*******************************************************************************/
s64 get_right_encoder(void)
{
	return g_encoder.right_encoder_sum;
}

/*******************************************************************************
  Function		: reset_encoder      
  Description	: reset encoder
  Input			: None                    
  Output		: None       
  Return		: None       
*******************************************************************************/
void reset_encoder(void)
{
	g_encoder.left_first_measure_flag = TRUE;
	g_encoder.right_first_measure_flag = TRUE;

	update_left_encoder();
	update_right_encoder();
}
/*******************************************************************************
  Function		: get_encoder_info      
  Description	: get encoder info
  Input			: None                    
  Output		: None       
  Return		: None       
*******************************************************************************/
void get_encoder_info(T_encoder * encoder)
{
	update_left_encoder();
	update_right_encoder();

	encoder->left_first_measure_flag = g_encoder.left_first_measure_flag;
	encoder->right_first_measure_flag = g_encoder.right_first_measure_flag;
	encoder->left_speed = g_encoder.left_speed;
	encoder->right_speed = g_encoder.right_speed ;
	encoder->left_encoder = g_encoder.left_encoder;
	encoder->right_encoder = g_encoder.right_encoder;
	encoder->last_left_encoder = g_encoder.last_left_encoder;
	encoder->last_right_encoder = g_encoder.last_right_encoder;
	encoder->left_encoder_overflow = g_encoder.left_encoder_overflow;
	encoder->right_encoder_overflow = g_encoder.right_encoder_overflow;
	encoder->left_encoder_delta = g_encoder.left_encoder_delta;
	encoder->right_encoder_delta = g_encoder.right_encoder_delta;
	encoder->left_encoder_sum = g_encoder.left_encoder_sum;
	encoder->right_encoder_sum = g_encoder.right_encoder_sum;
}
/* Copyright (C), 2017, TOPBAND Robot Team ************************************/

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

#ifndef _ENCODER_H_
#define _ENCODER_H_

/* includes *******************************************************************/
#include "stm32f4xx.h"
#include "typedef.h"


/* macros *********************************************************************/

#define ENCODER_LEFT_A_PORT						GPIOH
#define	ENCODER_LEFT_A_PIN						GPIO_Pin_9
#define	ENCODER_LEFT_A_CLK						RCC_AHB1Periph_GPIOH
#define	ENCODER_LEFT_A_PINSOURCE				GPIO_PinSource9

//#define ENCODER_LEFT_B_PORT					GPIOH
//#define ENCODER_LEFT_B_PIN					GPIO_Pin_X
//#define ENCODER_LEFT_B_CLK					RCC_AHB1Periph_GPIOH
//#define ENCODER_LEFT_B_PINSOURCE				GPIO_PinSourceX

#define ENCODER_RIGHT_A_PORT					GPIOA  					//GPIOE
#define	ENCODER_RIGHT_A_PIN						GPIO_Pin_2				//GPIO_Pin_7
#define	ENCODER_RIGHT_A_CLK						RCC_AHB1Periph_GPIOA 	//RCC_AHB1Periph_GPIOE
#define	ENCODER_RIGHT_A_PINSOURCE				GPIO_PinSource2 		//GPIO_PinSource7

//#define ENCODER_RIGHT_B_PORT					GPIOA
//#define ENCODER_RIGHT_B_PIN					GPIO_Pin_3
//#define ENCODER_RIGHT_B_CLK					RCC_AHB1Periph_GPIOA
//#define ENCODER_RIGHT_B_PINSOURCE				GPIO_PinSource2

#define ENCODER_LEFT_TIM_CLK			RCC_APB1Periph_TIM12
#define	ENCODER_RIGHT_TIM_CLK			RCC_APB2Periph_TIM9 	// RCC_APB2Periph_TIM11

#define ENCODER_LEFT_TIMER			TIM12
#define	ENCODER_RIGHT_TIMER			TIM9			// tim9

#define ENCODER_PPR					(s32)4
#define ENCODER_UPDATE_FREQUENCY	(s32)250

typedef struct
{
	T_bool left_first_measure_flag;
	T_bool right_first_measure_flag;
	s32	 left_speed;
	s32	 right_speed;
	s32	 left_encoder;
	s32	 right_encoder;
	s32  last_left_encoder;
	s32  last_right_encoder;
	s32  left_encoder_overflow;
	s32  right_encoder_overflow;
	s32  left_encoder_delta;
	s32  right_encoder_delta;
	s32  left_encoder_sum;
	s32  right_encoder_sum;
}T_encoder;

/* funcitons ******************************************************************/
void encoder_init(void);
s32 get_left_motor_speed(void);
s32 get_right_motor_speed(void);
s64 get_left_encoder(void);
s64 get_right_encoder(void);
void reset_encoder(void);
void get_encoder_info(T_encoder *encoder);


#endif

/* Copyright (C), 2017, TOPBAND Robot Team ************************************/

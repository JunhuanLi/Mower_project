/*******************************************************************************

  Copyright (C), 2017-2027, TOPBAND Co., Ltd.
  File name		: beep.h
  Author		: klaus     
  Version		: V1.0.0      
  Date			: 2017/05/17
  Description	: beep 
  
  History:        
                  
	1. Date			:
	   Author		:
	   Modification	:
	2. ...
    
*******************************************************************************/

#ifndef _BEEP_H_
#define _BEEP_H_


/* includes *******************************************************************/
#include "stm32f4xx.h"
#include "typedef.h"

/* macros *********************************************************************/
#define BEEP_PORT										GPIOH
#define	BEEP_PIN										GPIO_Pin_2
#define	BEEP_CLK										RCC_AHB1Periph_GPIOH

/* funcitons ******************************************************************/
void beep_init(void);
void reset_beep(void);
void get_beep_info(T_bool *beep);

void disable_beep(void);
void enable_beep(void);

#endif

/* Copyright (C), 2017-2027, TOPBAND Co., Ltd. ********************************/

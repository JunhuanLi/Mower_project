/*******************************************************************************
Copyright (C), 2017, TOPBAND Robot Team
File name		:	ist8310.c
Author			:	klaus
Version			:	V1.0.0	
Date			:	2017/05/23
Description		:	ist8310 driiver
*******************************************************************************/
#ifndef _IIC_H_
#define _IIC_H_

/* includes *******************************************************************/
#include "stm32f4xx.h"

/* macros *********************************************************************/
#define I2C_SCL_PORT	GPIOB
#define I2C_SCL_PIN		GPIO_Pin_6
#define I2C_SCL_CLK		RCC_AHB1Periph_GPIOB

#define I2C_SDA_PORT	GPIOB
#define I2C_SDA_PIN		GPIO_Pin_7
#define I2C_SDA_CLK		RCC_AHB1Periph_GPIOB

/* funcitons ******************************************************************/

void iic_init(void);           			 
void iic_start(void);				
void iic_stop(void);	  			
void iic_send_byte(u8 txd);			
u8 iic_read_byte(u8 ack);
u8 iic_wait_ack(void); 				
void iic_ack(void);				
void iic_nack(void);				

#endif

/* Copyright (C), 2017, TOPBAND Robot Team ************************************/


#ifndef __DEBUG_H__
#define __DEBUG_H__

#include <rtthread.h>
#include <rthw.h>
#include "stm32f4xx.h"
#include "lcd.h"

void debug_send(uint8_t* buf,uint8_t length);
void rt_debug(const char *buf,rt_size_t length);
//Kservice is changed rt_debug is added
//serial.c is changed comment line 226~line 230
#endif

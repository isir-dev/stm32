/*
  
* All rights reserved.
*
* File Name          : SignalsGeneration.h
* Description        : 信号源产生
*
* Version            : V1.0
* Author             : liu Yong
* Date               : 7/7/2012
*
* OriginalVersion    : V1.0
* Original Author    : liu Yong
* Date               : 7/7/2012
*/
#ifndef __SIGNALS_GENERATION__
#define __SIGNALS_GENERATION__

#include "stm32f4xx.h"


#define signal_sina_30HZ 1
#define signal_pulse_30HZ 2
#define signal_direct_current 3

#define signal_sina_7_8125HZ_776 4
#define signal_sina_7_8125HZ_97 5
#define Pulsewave12bit_2HZ_50 6
#define Pulsewave12bit_2HZ_75 7

void signals_generation_init(void);
void signals_ON(uint16_t type);
void signals_OFF(void);
void signals_generation_test(void);

#endif 

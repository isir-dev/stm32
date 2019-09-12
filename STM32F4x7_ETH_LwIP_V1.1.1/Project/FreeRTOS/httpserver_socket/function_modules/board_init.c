/*
  
* All rights reserved.
*
* File Name          : board_init.c
* Description        : ������Դ��ʼ��
*
* Version            : V1.0
* Author             : liu Yong
* Date               : 7/18/2012
*
* OriginalVersion    : V1.0
* Original Author    : liu Yong
* Date               : 7/18/2012
*/

#include "board_init.h"
#include "system_rcc_init.h"
#include "rs485.h"
#include "led.h"
#include "ads1255.h"
#include "sensor_test.h"
#include "mechanicalcontrol.h"
#include "direction.h"
#include "SignalsGeneration.h"
/*******************************************************************************
    ��������board_init
    ��  ��: 
    ��  ��:
    ����˵����������Դ��ʼ��
*/
void board_init(void)
{
    SysPer_RCC_Configuration();	 			/* Enable peripheral clock */
	  mechanicalcontrol_init();
    ppp_rs485_configuration();
    //ad_modules_init();
    init_direction();
	  signals_generation_init();
//		ADC_Config_RFLASH();
}


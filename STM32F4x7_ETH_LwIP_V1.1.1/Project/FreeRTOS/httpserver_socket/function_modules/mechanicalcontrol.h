/*
  
* All rights reserved.
*
* File Name          : mechanicalcontrol.h
* Description        : 机械控制部分函数
*
* Version            : V1.0
* Author             : liu Yong
* Date               : 7/7/2012
*
* OriginalVersion    : V1.0
* Original Author    : liu Yong
* Date               : 7/7/2012
*/
#ifndef __MECHANICAL_CONTROL_HEADER__
#define __MECHANICAL_CONTROL_HEADER__

#include "stm32f4xx.h"
#include "board_init.h"

#define MotCtrl_TIMER 5

#define MotCtrl_1		GPIO_Pin_0		   	/* PA0 */ 
#define MotCtrl_2		GPIO_Pin_1	   		/* PA1 */
#define Mechanical_exit		GPIO_Pin_0 /* PC0，电机运动检测中断 */

#define Mot_Running		0x02	
#define Mot_stoped		0x01

#define mot_power_on GPIO_SetBits(GPIOA , MotCtrl_2)
#define mot_power_off GPIO_ResetBits(GPIOA , MotCtrl_2)
#define mot_foward GPIO_SetBits(GPIOA , MotCtrl_1)
#define mot_reverse GPIO_ResetBits(GPIOA , MotCtrl_1)

//机械控制内部函数
void MecConPortConfig(void);
void Mechanical_exit_init(void);
void mechanical_exit_control(FunctionalState state);

//机械控制外部调用接口
void mechanicalcontrol_init(void);
void mechanical_exit_irq(void);
void mechanicalcontrol_test(void);
void mot_stretch(void);
void mot_approach(void);
void mot_main_monitor(void);

void mechanical_init(void);

__IO extern uint8_t mot_run_state;//电机工作状态
#endif

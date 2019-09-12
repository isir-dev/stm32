/*
  
* All rights reserved.
*
* File Name          : mechanicalcontrol.c
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
/* Scheduler includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "mechanicalcontrol.h"

#define MECHSERVER_THREAD_PRIO    ( tskIDLE_PRIORITY + 5 )
__IO uint8_t mot_run_state = Mot_stoped;//电机工作状态
__IO uint8_t mot_switch_available = ENABLE;//电机开关使能//DISABLE
//***************************************************************************
void delay_s(int s)
{
	int i,j,k;
	for(k=0;k<s;k++){
		for(i=0;i<168;i++)
			{
				for(j=0;j<250000;j++);
			}
		}
}

void delay_ms(int ms)
{
	int i,j,k;
	for(k=0;k<ms;k++){
		for(i=0;i<168;i++)
		{
			for(j=0;j<250;j++);
		}
	}

}

void delay_us(int us)
{
	int i,j;
	for(i=0;i<us;i++){
		for(j=0;j<42;j++);
	}
}

/*******************************************************************************
    函数名：MecConPortConfig
    输  入: 
    输  出:
    功能说明：机械部分控制端口初始化 
*/
void MecConPortConfig(void)
{
    GPIO_InitTypeDef GPIO_InitType;	              	/* GPIO端口结构体变量 */
    
    /************************控制端口配置*************************/
    GPIO_InitType.GPIO_Pin = MotCtrl_1|MotCtrl_2;		/* 电机控制端口 */ 	
    GPIO_InitType.GPIO_Speed = GPIO_Speed_25MHz;	    /* 口线翻转速度为25MHz */
    GPIO_InitType.GPIO_Mode = GPIO_Mode_OUT;  
    GPIO_InitType.GPIO_OType = GPIO_OType_PP;	  		/* 推挽输出	*/
    GPIO_Init(GPIOA, &GPIO_InitType);
}

/*******************************************************************************
    函数名：Mechanical_exit_init
    输  入: 
    输  出:
    功能说明：电机检测中断配置
*/
void Mechanical_exit_init(void)
{
    NVIC_InitTypeDef NVIC_InitStruct;
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    
    /*--------------------------------------------------------------------------------*/
    GPIO_InitStructure.GPIO_Pin 	= Mechanical_exit; 			/* 电机检测中断*/	    
    GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN;	        /* Input mode*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	    /* speed 50MHz */
    //GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;			/* NOPULL input */
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    /*-------------- Select the input source pin for the EXTI line --------------*/
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);		/* PC0-EXIT外部中断 */
    
    /*----------- Select the mode and configure the trigger selection	-----------*/
    /* Configure EXTI Line0 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    /*------------------------------ 中断初始化 ---------------------------------*/ 
    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;	    		/* EXTI Line0 Interrupt	*/
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;	/* 先占优先级为1 */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;			/* 总优先级为1 */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;				/* 打开通道中断	*/
    NVIC_Init(&NVIC_InitStruct);
}

/*******************************************************************************
    函数名：mechanicalcontrol_init
    输  入: 
    输  出:
    功能说明：机械控制初始化
*/
void mechanicalcontrol_init(void)
{
    /***************************** 机械控制端口配置 ********************************/ 
    MecConPortConfig(); 						/* 机械控制端口配置 */
    mot_power_off;
    Mechanical_exit_init();
    //mot_power_on;
    mot_run_state = Mot_stoped;
	  mot_switch_available = ENABLE;
}
/*******************************************************************************
    函数名：mot_stretch
    输  入: 
    输  出:
    功能说明：机械臂伸展
*/
void mot_stretch(void)
{
    mot_switch_available = DISABLE;
    mechanical_exit_control(DISABLE); //禁能电机中断50ms,防止毛刺
	  mot_reverse;//反转
	  mot_power_on;
	  mot_run_state = Mot_Running;
	  vTaskDelay(800);
	  mechanical_exit_control(ENABLE); //
	  mot_switch_available = ENABLE;
}
/******* ************************************************************************
    函数名：mot_approach
    输  入: 
    输  出:
    功能说明：机械臂收缩
*/
void mot_approach(void)
{
	  mot_switch_available = DISABLE;
    mechanical_exit_control(DISABLE); //禁能电机中断50ms,防止毛刺
    mot_foward;//正转
	  mot_power_on;
	  mot_run_state = Mot_Running;
	  vTaskDelay(800);
	  mechanical_exit_control(ENABLE); //禁能电机中断50ms,防止毛刺
	  mot_switch_available = ENABLE;
}
/*******************************************************************************
    函数名：mechanical_exit_irq
    输  入: 
    输  出:
    功能说明：电机控制中断函数
*/
void mechanical_exit_irq(void)
{
//    mot_power_off;
//    mot_run_state = Mot_stoped;
		delay_us(1);
		if(GPIO_ReadInputDataBit(GPIOC, Mechanical_exit)==1)
		{
			mot_power_off;
			mot_run_state = Mot_stoped;
		}	
}
/*******************************************************************************
    函数名：mechanical_exit_control
    输  入: 
    输  出:
    功能说明：电机状态监测启动开关启动或停止
*/
void mechanical_exit_control(FunctionalState state)
{
    EXTI_InitTypeDef   EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; 
    //EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;		
    EXTI_InitStructure.EXTI_LineCmd = state;
    EXTI_Init(&EXTI_InitStructure);
}
//.....test....
///*******************************************************************************
//    函数名：mechanicalcontrol_test
//    输  入: 
//    输  出:
//    功能说明：机械控制测试
//*/
//void mechanicalcontrol_test(void)
//{
//    //float AD_value=0;
//    mot_power_off;
//    mot_stretch();//伸展
//    mot_approach();//收缩
//    mot_power_on;
//}
/**
  * @brief  mechanical server thread 
  * @param arg: pointer on argument(not used here) 
  * @retval None
  */
static void mechanical_thread(void *arg)
{
	u8 scan_bit;   //单个按键扫描变量
  while (1) 
  {
			scan_bit = GPIO_ReadInputDataBit(GPIOC, Mechanical_exit);
			if(scan_bit==1)
			{	
				vTaskDelay(10);
				scan_bit = GPIO_ReadInputDataBit(GPIOC, Mechanical_exit);
				if((0x01 == scan_bit) && (mot_switch_available == ENABLE))
				{	
					mot_power_off;
					mot_run_state = Mot_stoped;
				}				
			}
			vTaskDelay(500);
  }
}
//    函数名：mechanical_init
//    输  入: 
//    输  出:
//    功能说明：
//*/
void mechanical_init()
{
		xTaskCreate(mechanical_thread, (int8_t *) "Mechani", configMINIMAL_STACK_SIZE * 1, NULL,MECHSERVER_THREAD_PRIO, NULL);
}

void EXTI0_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        mechanical_exit_irq();
        
        /* Clear the EXTI line 0 pending bit */
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

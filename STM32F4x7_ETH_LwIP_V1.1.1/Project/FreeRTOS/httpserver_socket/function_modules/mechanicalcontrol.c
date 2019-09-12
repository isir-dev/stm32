/*
  
* All rights reserved.
*
* File Name          : mechanicalcontrol.c
* Description        : ��е���Ʋ��ֺ���
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
__IO uint8_t mot_run_state = Mot_stoped;//�������״̬
__IO uint8_t mot_switch_available = ENABLE;//�������ʹ��//DISABLE
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
    ��������MecConPortConfig
    ��  ��: 
    ��  ��:
    ����˵������е���ֿ��ƶ˿ڳ�ʼ�� 
*/
void MecConPortConfig(void)
{
    GPIO_InitTypeDef GPIO_InitType;	              	/* GPIO�˿ڽṹ����� */
    
    /************************���ƶ˿�����*************************/
    GPIO_InitType.GPIO_Pin = MotCtrl_1|MotCtrl_2;		/* ������ƶ˿� */ 	
    GPIO_InitType.GPIO_Speed = GPIO_Speed_25MHz;	    /* ���߷�ת�ٶ�Ϊ25MHz */
    GPIO_InitType.GPIO_Mode = GPIO_Mode_OUT;  
    GPIO_InitType.GPIO_OType = GPIO_OType_PP;	  		/* �������	*/
    GPIO_Init(GPIOA, &GPIO_InitType);
}

/*******************************************************************************
    ��������Mechanical_exit_init
    ��  ��: 
    ��  ��:
    ����˵�����������ж�����
*/
void Mechanical_exit_init(void)
{
    NVIC_InitTypeDef NVIC_InitStruct;
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    
    /*--------------------------------------------------------------------------------*/
    GPIO_InitStructure.GPIO_Pin 	= Mechanical_exit; 			/* �������ж�*/	    
    GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN;	        /* Input mode*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	    /* speed 50MHz */
    //GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;			/* NOPULL input */
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    /*-------------- Select the input source pin for the EXTI line --------------*/
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);		/* PC0-EXIT�ⲿ�ж� */
    
    /*----------- Select the mode and configure the trigger selection	-----------*/
    /* Configure EXTI Line0 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    /*------------------------------ �жϳ�ʼ�� ---------------------------------*/ 
    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;	    		/* EXTI Line0 Interrupt	*/
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;	/* ��ռ���ȼ�Ϊ1 */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;			/* �����ȼ�Ϊ1 */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;				/* ��ͨ���ж�	*/
    NVIC_Init(&NVIC_InitStruct);
}

/*******************************************************************************
    ��������mechanicalcontrol_init
    ��  ��: 
    ��  ��:
    ����˵������е���Ƴ�ʼ��
*/
void mechanicalcontrol_init(void)
{
    /***************************** ��е���ƶ˿����� ********************************/ 
    MecConPortConfig(); 						/* ��е���ƶ˿����� */
    mot_power_off;
    Mechanical_exit_init();
    //mot_power_on;
    mot_run_state = Mot_stoped;
	  mot_switch_available = ENABLE;
}
/*******************************************************************************
    ��������mot_stretch
    ��  ��: 
    ��  ��:
    ����˵������е����չ
*/
void mot_stretch(void)
{
    mot_switch_available = DISABLE;
    mechanical_exit_control(DISABLE); //���ܵ���ж�50ms,��ֹë��
	  mot_reverse;//��ת
	  mot_power_on;
	  mot_run_state = Mot_Running;
	  vTaskDelay(800);
	  mechanical_exit_control(ENABLE); //
	  mot_switch_available = ENABLE;
}
/******* ************************************************************************
    ��������mot_approach
    ��  ��: 
    ��  ��:
    ����˵������е������
*/
void mot_approach(void)
{
	  mot_switch_available = DISABLE;
    mechanical_exit_control(DISABLE); //���ܵ���ж�50ms,��ֹë��
    mot_foward;//��ת
	  mot_power_on;
	  mot_run_state = Mot_Running;
	  vTaskDelay(800);
	  mechanical_exit_control(ENABLE); //���ܵ���ж�50ms,��ֹë��
	  mot_switch_available = ENABLE;
}
/*******************************************************************************
    ��������mechanical_exit_irq
    ��  ��: 
    ��  ��:
    ����˵������������жϺ���
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
    ��������mechanical_exit_control
    ��  ��: 
    ��  ��:
    ����˵�������״̬�����������������ֹͣ
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
//    ��������mechanicalcontrol_test
//    ��  ��: 
//    ��  ��:
//    ����˵������е���Ʋ���
//*/
//void mechanicalcontrol_test(void)
//{
//    //float AD_value=0;
//    mot_power_off;
//    mot_stretch();//��չ
//    mot_approach();//����
//    mot_power_on;
//}
/**
  * @brief  mechanical server thread 
  * @param arg: pointer on argument(not used here) 
  * @retval None
  */
static void mechanical_thread(void *arg)
{
	u8 scan_bit;   //��������ɨ�����
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
//    ��������mechanical_init
//    ��  ��: 
//    ��  ��:
//    ����˵����
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

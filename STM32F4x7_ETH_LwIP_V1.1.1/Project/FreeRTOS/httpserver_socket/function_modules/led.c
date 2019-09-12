/*
  
* All rights reserved.
*
* File Name          : led.c
* Description        :  
*
* Version            : V1.0
* Author             : liu Yong
* Date               : 10/28/2015
*
* OriginalVersion    : V1.0
* Original Author    : liu Yong
* Date               : 10/28/2015
*/
  
/* Includes ------------------------------------------------------------------*/
#include "led.h"

/**
  * @brief  Configures LED GPIO.
  * @param  None
  * @retval None
  */
void STM_EVAL_LEDInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIO_LED Clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//|GPIO_Pin_10;	
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;	    
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;  
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
  * @brief  Turns selected LED On.
  * @param  None
  * @retval None
  */
void STM_EVAL_LEDOn( void )
{
  GPIOB->BSRRL = GPIO_Pin_11;
}

/**
  * @brief  Turns selected LED Off.
  * @param  None
  * @retval None
  */
void STM_EVAL_LEDOff( void )
{
  GPIOB->BSRRH = GPIO_Pin_11;  
}

/**
  * @brief  Toggles the  LED.
  * @param  None
  * @retval None
  */
void STM_EVAL_LEDToggle( void )
{
  GPIOB->ODR ^= GPIO_Pin_11;
}


/**
  * @brief  Initializes the  LEDs resources.
  * @param  None
  * @retval None
  */
void LED_Init(void)
{
  /* Initialize LEDs */
  STM_EVAL_LEDInit();
}
    
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/*
  
* All rights reserved.
*
* File Name          : led.h
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LED_H
#define __LED_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

#define LED_OFF STM_EVAL_LEDOn()
#define LED_ON STM_EVAL_LEDOff()

void STM_EVAL_LEDInit(void);
void STM_EVAL_LEDOn(void);
void STM_EVAL_LEDOff(void);
void STM_EVAL_LEDToggle(void);
void LED_Init(void);
/**
  * @}
  */
  
#ifdef __cplusplus
}
#endif

#endif /* __LED_H */
/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

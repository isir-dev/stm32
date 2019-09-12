/*
  
* All rights reserved.
*
* File Name          : System_RCC_Configration.c
* Description        : 
*
* Version            : V1.0
* Author             : liu Yong
* Date               : 7/18/2012
*
* OriginalVersion    : V1.0
* Original Author    : liu Yong
* Date               : 7/18/2012
*/

#include "system_rcc_init.h"
//============================================================================//
/*************************System clock configuration*************************/
//============================================================================//void Sys_RCC_Configuration(void)
//void Sys_RCC_Configuration(void)
//{ 
//    ErrorStatus HSEStartUpStatus;				  /* Define a state detection variables */
//    
//    RCC_DeInit(); 			                  /* Resets the RCC clock configuration to the default reset state */
//    RCC_HSEConfig(RCC_HSE_ON);            /* Configures the External High Speed oscillator (HSE) */
//    HSEStartUpStatus = RCC_WaitForHSEStartUp(); /* Pick up the state of HSE */
//    
//    RCC_PLLCmd(DISABLE);						  /* Disables the main PLL */
//    RCC_PLLI2SCmd(DISABLE);					  /* Disables the PLLI2S */
//    
//    if(HSEStartUpStatus == SUCCESS)			  /* Waits for HSE stable */
//    {
//        RCC_HCLKConfig(RCC_SYSCLK_Div1); 		  /* Configures the AHB clock (HCLK), equal System Clock */
//        RCC_PCLK2Config(RCC_HCLK_Div2);       /* Configures the High Speed APB clock (PCLK2)£¨APB2,PCLK2£©, (1/2)*AHB_Clock */
//        RCC_PCLK1Config(RCC_HCLK_Div2);			  /* Configures the Low Speed APB clock (PCLK1)£¨APB1,PCLK1£©,(1/2)*AHB_Clock,16M */
//        
//        /**********************************************************************************/
//        FLASH_SetLatency(FLASH_Latency_1);		  /* Sets the code latency value is one system clock */
//        FLASH_PrefetchBufferCmd(ENABLE);  		  /* Enables or disables the Prefetch Buffer */
//        /**********************************************************************************/
//        
//        RCC_PLLConfig(RCC_PLLSource_HSE, 4, 64, 4, 2);   /* 64M SYSCLK CLOCK */
//			  //RCC_PLLConfig(RCC_PLLSource_HSE, 16, 336, 2, 7);   /* 168M SYSCLK CLOCK */
//        
//        RCC_PLLCmd(ENABLE);	                      /* Enable main PLL */
//        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);    /* Wait for main PLL clock ready */
//        
//        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);   /* PLL selected as system clock source */
//        while(RCC_GetSYSCLKSource() != 0x08);	  /* Wait for PLL used as system clock successfully */
//    }
//    RCC_HSICmd(DISABLE);						  /* Disables the Internal High Speed oscillator (HSI), reduce the consumption of energy */
//    RCC_LSICmd(DISABLE);						  /* Disables the Internal Low Speed oscillator (LSI) */
//    RCC_RTCCLKCmd(DISABLE);					  /* Disables the RTC clock */
//    RCC_ClockSecuritySystemCmd(DISABLE);		  /* Disables the Clock Security System */
//}

//============================================================================//
/************************** Enable peripheral clock *************************/
//============================================================================//
void SysPer_RCC_Configuration(void)
{
    /*----------------Enable clock of GPIOA, GPIOB, GPIOC,and GPIOD-----------------*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_DMA1, ENABLE);
    
    /*-----------------Enable clock of USART1, SPI1 and ADC1------------------*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_SPI1, ENABLE);
    
    /*----------------Enable clock of SPI2 and SPI3 and Timer2----------------*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2|RCC_APB1Periph_SPI3|RCC_APB1Periph_TIM2|RCC_APB1Periph_DAC | RCC_APB1Periph_TIM6, ENABLE);
    
    /*------------------------- Enable ETHERNET clock ------------------------*/
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_ETH_MAC | RCC_AHB1Periph_ETH_MAC_Tx | RCC_AHB1Periph_ETH_MAC_Rx, ENABLE);
    
    /*-------------------- Enable clock of DMA1 and DMA2 ---------------------*/
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    
    /*------------------------ Enable SYSCFG APB clock -----------------------*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
}

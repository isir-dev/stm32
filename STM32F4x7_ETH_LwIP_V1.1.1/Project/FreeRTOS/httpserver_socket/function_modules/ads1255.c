/*
  
* All rights reserved.
*
* File Name          : ADS1255.c
* Description        : ADS1255读写控制及配置相关函数
*
* Version            : V1.0
* Author             : liu Yong
* Date               : 10/31/2015
*
* OriginalVersion    : V1.0
* Original Author    : liu Yong
* Date               : 10/31/2015
*/

/* Scheduler includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "ads1255.h"
#include "sensor_test.h"
#include "main.h"
#include "uart.h"
#include "rs485.h"
#include "string.h"
#include "malloc.h"
#include "direction.h"
#include "SignalsGeneration.h"
#include "stmflash.h"

#define ADCLOAD_THREAD_PRIO    ( tskIDLE_PRIORITY + 4 )
#define ADCPRE_THREAD_PRIO    ( tskIDLE_PRIORITY + 4 )

/*===============================全局外部变量定义======================================*/
__IO uint8_t ADS1255_GAIN = ADS1255_GAIN_1;				 	/* ADC增益控制变量 */
__IO uint8_t ADS1255_DRATEVALUE = ADS1256_DRATE_3750SPS; 			/* ADC采样率控制变量 */
uint8_t *sample_data;
PreSamplingData pre_sample;
int remainNumPoints;
SETS sets;
/*******************************************************************************
    函数名：Delay_us
    输  入: 
    输  出:
    功能说明：ADC所需Delay_us
*/
void Delay_us(s32 time_us)
{    
   u16 i=0;  
   while(time_us--)
   {
      //i=32*10;  
		  i=168*20;  
      while(i--) ;    
   }
}
/*******************************************************************************
    函数名：Sample_Data_init
    输  入: 
    输  出:
    功能说明：
*/
uint8_t Sample_Data_init(void)
{
	sample_data=(uint8_t *)mymalloc(SRAMIN,2048*3*3);
	if(sample_data==NULL) 
	{
		return 1;
	}
	else 
	{
		return 0;
	}
}

/**************************************************************
函数名：sensor_test_switch
输入：
输出：
功能说明：传感器测试引脚开关
***************************************************************/
void sensor_test_switch(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;	/* GPIO port struct */
	
		GPIO_InitStructure.GPIO_Pin = PC8_SENSOR_CHECK; 	  	  		  /* 光耦开关*/	    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	              		  /* 输出 mode*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;	                  /* speed 50MHz */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;					  /* Push-pull mode */
    GPIO_Init(GPIOC, &GPIO_InitStructure);	
}

/*******************************************************************************
    函数名：ADC_SPI_Port_Configuration
    输  入: 
    输  出:
    功能说明：ADC所需GPIO口配置
*/
void ADC_SPI_Port_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;	/* GPIO port struct */
    /*-----------------------------X-axis/SPI1 Confiuration----------------------------*/
    GPIO_InitStructure.GPIO_Pin = Sensor_X_NDRDY; 	  	  /* DRDY Configuration*/	    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	              		  /* Input mode*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;	                  /* speed 50MHz */
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;					  	  /* PULL up */
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = Sensor_X_DOUT|Sensor_X_SCLK|Sensor_X_DIN;/* MISO、SCLK、MOSI Configuration */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	                      /* Must be set as reuse push-pull output */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;	                  /* speed 50MHz */
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;					  /* Push-pull mode */
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /*----------------Y-axis/SPI2 and Sensor_Z_NDRDY Configuration--------------------*/
    GPIO_InitStructure.GPIO_Pin = Sensor_Y_NDRDY|Sensor_Z_NDRDY; /* DRDY Configuration */	    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	              		  /* Input mode*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;	                  /* speed 50MHz */
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;					  	  /* NOPULL input */
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = Sensor_Y_DOUT|Sensor_Y_SCLK|Sensor_Y_DIN;/* SCLK、MOSI Configuration */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	                      /* Must be set as reuse push-pull output */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;	                  /* speed 50MHz */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;					  /* Push-pull mode */
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /*----------------------------Z-axis/SPI3 Configuration---------------------------*/ 			                  
    GPIO_InitStructure.GPIO_Pin = Sensor_Z_DOUT|Sensor_Z_SCLK | Sensor_Z_DIN;	  	  /* DRDY Configuration */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	                      /* Must be set as reuse push-pull output */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;	                  /* speed 50MHz */
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;					  /* Push-pull mode */
    GPIO_Init(GPIOC, &GPIO_InitStructure);
		
		/*-------------------------------Alternate function--------------------------------*/
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
    
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
    
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);
    
    /*----------------------Synchronization port configuration-----------------------*/
    GPIO_InitStructure.GPIO_Pin = ADC_SYNC; 	  	  		  /* Synchronization*/	    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	              		  /* Input mode*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;	                  /* speed 50MHz */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;					  /* Push-pull mode */
    GPIO_Init(GPIOA, &GPIO_InitStructure);	 
		
		
		    /*----------------------Synchronization port configuration-----------------------*/
    GPIO_InitStructure.GPIO_Pin = Sensor_Reset; 	  	  		  /* Synchronization*/	    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	              		  /* Input mode*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;	                  /* speed 50MHz */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;					  /* Push-pull mode */
    GPIO_Init(GPIOC, &GPIO_InitStructure);	
		
//		Sensor_Reset_OFF;
//		Delay_us(100);
//		Sensor_Reset_ON;
//		Delay_us(100);
		
		
}

/*******************************************************************************
    函数名：ADC_SPI_Init
    输  入: 
    输  出:
    功能说明：ADC所需SPI初始化
*/
void ADC_SPI_Init(void)
{
    SPI_InitTypeDef SPI_InitStructure; 
	  
	  RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);//??SPI1
	  RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);//????SPI1
	  RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,ENABLE);//??SPI2
	  RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,DISABLE);//????SPI2
	  RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,ENABLE);//??SPI3
	  RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,DISABLE);//????SPI3
    
    /* -----------------------------SPIx Configuration ------------------------------*/
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  /* FullDuplex mode */
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						  /* Master mode */
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					  /*	8 bits mode */
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		/*Clock polarity，low electrical level while idle condition */
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;		/* Catch up the data while the second clock edge coming */
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;			/* Soft mode */
	  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; /* Baud rate configuration，BR=84M/64， */
	  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;/* Specifies data transfers start from MSB */
    SPI_InitStructure.SPI_CRCPolynomial = 7;			/* CRC checking polynomial choice */
    
    SPI_Init(Sensor_X, &SPI_InitStructure);			/* Sensor_X/SPI1 initialization	*/
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32; /* Baud rate configuration，BR=42M/32， */
    SPI_Init(Sensor_Y, &SPI_InitStructure);			/* Sensor_Y/SPI2 initialization	*/
    SPI_Init(Sensor_Z, &SPI_InitStructure);			/* Sensor_Z/SPI3 initialization	*/
    
    ADC_SYNC_DEFI_STATE;							/* initialize into high level */
    Delay_us(3600);								/* wait for the clock stable */
}

/*******************************************************************************
    函数名：AllSPI_Enable
    输  入: 
    输  出:
    功能说明：Enables the specified SPI peripheral
*/
void AllSPI_Enable(void)
{
	  #ifdef Stack_X
	  SPI_Cmd(Sensor_X, ENABLE); //
    #endif
	  #ifdef Stack_Y
	  SPI_Cmd(Sensor_Y, ENABLE); //
	  #endif
    #ifdef Stack_Z
	  SPI_Cmd(Sensor_Z, ENABLE); //
    #endif
    Delay_us(100);  			/* 等待外设稳定 */
}

/*******************************************************************************
    函数名：AllSPI_Disenable
    输  入: 
    输  出:
    功能说明：Disables the specified SPI peripheral
*/
void AllSPI_Disenable(void)
{
    #ifdef Stack_X
	  SPI_Cmd(Sensor_X, DISABLE); //
	  #endif
    #ifdef Stack_Y
	  SPI_Cmd(Sensor_Y, DISABLE); //
    #endif
	  #ifdef Stack_Z
	  SPI_Cmd(Sensor_Z, DISABLE); //
  	#endif
}
/***********************************************************************************/

//AD主要操作函数
/*******************************************************************************
    函数名：AD_RD_Reg
    输  入: 
    输  出:
    功能说明：read only one register
*/
uint8_t AD_RD_Reg(SPI_TypeDef* SPIx, uint8_t reg)
{
    while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(SPIx, ADS1255_CMD_RREG+reg); 
    
		while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(SPIx, 0x00); 
    Delay_us(10); 									/* delay time more than 6.52uS */
    
    while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(SPIx, 0x00); 
    
		while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 
		//Delay_us(10);
		return SPI_I2S_ReceiveData(SPIx); 
}


/*******************************************************************************
    函数名：AD_WR_Reg
    输  入: 
    输  出:
    功能说明：write one registe
*/
void AD_WR_Reg(SPI_TypeDef* SPIx, uint8_t reg, uint8_t wdata)
{
    while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(SPIx, ADS1255_CMD_WREG+reg); 
			
		while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(SPIx, 0x00); 
    
		while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
    SPI_I2S_SendData(SPIx, wdata); 			/* write data into the target register */
    Delay_us(10);
}


/*******************************************************************************
    函数名：AD_RD_MultiReg
    输  入: 
    输  出:
    功能说明：read multiply register
*/
void AD_RD_MultiReg(SPI_TypeDef* SPIx, uint8_t reg, uint8_t regnum, uint8_t *rBuf)  /* read data fromm a group of registers and put them into the buffer rBuf */
{      
    uint8_t Loop_R;
    while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(SPIx, ADS1255_CMD_RREG+reg); 
    
    while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(SPIx, regnum-1); 
    Delay_us(10); 									/* delay time more than 6.52uS */
    
    for(Loop_R=0;Loop_R<regnum;Loop_R++)
    {
				while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
				SPI_I2S_SendData(SPIx, 0x00); 
				while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} 
				rBuf[Loop_R]=SPI_I2S_ReceiveData(SPIx); 
    } 
}


/*******************************************************************************
    函数名：AD_WR_MultiReg
    输  入: 
    输  出:
    功能说明：write multiply register
*/
void AD_WR_MultiReg(SPI_TypeDef* SPIx, uint8_t reg, uint8_t regnum, uint8_t *wBuf)  /* write data into a group of registers */
{    
    uint8_t Loop_W; 
    while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(SPIx, ADS1255_CMD_WREG+reg); 
    
    while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(SPIx, regnum-1); 
    
    for(Loop_W=0;Loop_W<=regnum;Loop_W++)
    {
				while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
				SPI_I2S_SendData(SPIx, wBuf[Loop_W]); 
    }
}

/*******************************************************************************
    函数名：AD_Config
    输  入: 
    输  出:
    功能说明：ADS1255 configuration (default)
*/
void ADC_Parameter_Config(void)   /* initialize ADS1255 with the default configuration	*/
{
	  #ifdef Stack_X
    AD_WR_Reg(Sensor_X, ADS1255_DRATE, (u8)ADS1255_DRATEVALUE);
	  AD_WR_Reg(Sensor_X, ADS1255_ADCON, ADS1255_GAIN);
    AD_WR_Reg(Sensor_X, ADS1255_STATUS, 0x05); /* STATUS REGISTER --> Most Significant Bit First,Auto-Calibration Enabled,Buffer Disabled */
    #endif
	  #ifdef Stack_Y
	  AD_WR_Reg(Sensor_Y, ADS1255_DRATE, (u8)ADS1255_DRATEVALUE);
	  AD_WR_Reg(Sensor_Y, ADS1255_ADCON, ADS1255_GAIN);
	  AD_WR_Reg(Sensor_Y, ADS1255_STATUS, 0x05);
	  #endif
	  #ifdef Stack_Z
    AD_WR_Reg(Sensor_Z, ADS1255_DRATE, (u8)ADS1255_DRATEVALUE);
	  AD_WR_Reg(Sensor_Z, ADS1255_ADCON, ADS1255_GAIN);
	  AD_WR_Reg(Sensor_Z, ADS1255_STATUS, 0x05);
	  #endif
	  //Self_Cal();          /* self Gain and offset offset correction	*/
    //Sys_OCal();          /* System offset correction */
	  //Sys_HCal();          /* System gain correction */
}

/*******************************************************************************
    函数名：ADC_Parameter_Config_Selftest_start
    输  入: 
    输  出:
    功能说明：ADS1255 configuration (default)
*/
void ADC_Parameter_Config_Selftest_start(void)   /* initialize ADS1255 with the default configuration	*/
{
	  #ifdef Stack_X
    AD_WR_Reg(Sensor_X, ADS1255_DRATE, (u8)ADS1255_DRATEVALUE);
	  AD_WR_Reg(Sensor_X, ADS1255_ADCON, ADS1255_GAIN_1);
    AD_WR_Reg(Sensor_X, ADS1255_STATUS, 0x05); /* STATUS REGISTER --> Most Significant Bit First,Auto-Calibration Enabled,Buffer Disabled */
    #endif
	  #ifdef Stack_Y
	  AD_WR_Reg(Sensor_Y, ADS1255_DRATE, (u8)ADS1255_DRATEVALUE);
	  AD_WR_Reg(Sensor_Y, ADS1255_ADCON, ADS1255_GAIN_1);
	  AD_WR_Reg(Sensor_Y, ADS1255_STATUS, 0x05);
	  #endif
	  #ifdef Stack_Z
    AD_WR_Reg(Sensor_Z, ADS1255_DRATE, (u8)ADS1255_DRATEVALUE);
	  AD_WR_Reg(Sensor_Z, ADS1255_ADCON, ADS1255_GAIN_1);
	  AD_WR_Reg(Sensor_Z, ADS1255_STATUS, 0x05);
	  #endif
	  //Self_Cal();          /* self Gain and offset offset correction	*/
    //Sys_OCal();          /* System offset correction */
	  //Sys_HCal();          /* System gain correction */
}
/*******************************************************************************
    函数名：Stop_RDC
    输  入: 
    输  出:
    功能说明:Stop read  Data from AD continuous
*/
void Stop_RDC(void)
{
    /****************向ADC_Sensor_X发送停止连续读取数据指令******************/
    #ifdef Stack_X
		while(GPIO_ReadInputDataBit(GPIOC,Sensor_X_NDRDY)){}; /* read data while DRDY is low */
    while(SPI_I2S_GetFlagStatus(Sensor_X, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(Sensor_X, ADS1255_CMD_SDATAC); 
    #endif
    /****************向ADC_Sensor_Y发送停止连续读取数据指令******************/
    #ifdef Stack_Y
		while(GPIO_ReadInputDataBit(GPIOB,Sensor_Y_NDRDY)){}; /* read data while DRDY is low */
		while(SPI_I2S_GetFlagStatus(Sensor_Y, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(Sensor_Y, ADS1255_CMD_SDATAC);
    #endif
    /****************向ADC_Sensor_Z发送停止连续读取数据指令******************/
    #ifdef Stack_Z
		while(GPIO_ReadInputDataBit(GPIOB,Sensor_Z_NDRDY)){}; /* read data while DRDY is low */
		while(SPI_I2S_GetFlagStatus(Sensor_Z, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(Sensor_Z, ADS1255_CMD_SDATAC);
    #endif	
}

/*******************************************************************************
    函数名：Self_Cal
    输  入: 
    输  出:
    功能说明:Self Offset and Gain Calibration
*/
void Self_Cal(void)
{
    #ifdef Stack_X
		while(GPIO_ReadInputDataBit(GPIOC,Sensor_X_NDRDY)){}; /* read data while DRDY is low */
    while(SPI_I2S_GetFlagStatus(Sensor_X, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(Sensor_X, ADS1255_CMD_SELFOCAL); 
    //Delay_us(50);							  	/* delay time more than 50us	*/
    #endif	
    #ifdef Stack_Y
	  while(GPIO_ReadInputDataBit(GPIOB,Sensor_Y_NDRDY)){}; /* read data while DRDY is low */
    while(SPI_I2S_GetFlagStatus(Sensor_Y, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(Sensor_Y, ADS1255_CMD_SELFOCAL); 
    //Delay_us(50);							  	/* delay time more than 50us	*/
    #endif	
    #ifdef Stack_Z 
	  while(GPIO_ReadInputDataBit(GPIOB,Sensor_Z_NDRDY)){}; /* read data while DRDY is low */
    while(SPI_I2S_GetFlagStatus(Sensor_Z, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(Sensor_Z, ADS1255_CMD_SELFOCAL); 
	  #endif	
		Delay_us(50);						 		/* delay time more than 50ms */
}

/*******************************************************************************
    函数名：Self_OCal
    输  入: 
    输  出:
    功能说明:offset self-calibration
*/
void Self_OCal(void)
{
    #ifdef Stack_X
	  while((GPIOC->IDR & Sensor_X_NDRDY)){};      		/* DRDY信号为低时发送连续读数据指令，如果同步做得好，可以不要 */
    while((Sensor_X->SR & SPI_I2S_FLAG_TXE) == 0);  	/* 等待发送寄存器为空 */
    Sensor_X->DR = ADS1255_CMD_SELFOCAL;              /* Send bias and gain self-calibration instructions */
    //Delay_us(50);						 		/* delay time more than 50ms */
    #endif
    #ifdef Stack_Y
	  while((GPIOB->IDR & Sensor_Y_NDRDY)){};      		/* DRDY信号为低时发送连续读数据指令，如果同步做得好，可以不要 */
    while((Sensor_Y->SR & SPI_I2S_FLAG_TXE) == 0);  	/* 等待发送寄存器为空 */
    Sensor_Y->DR = ADS1255_CMD_SELFOCAL;              /* Send bias and gain self-calibration instructions */
    //Delay_us(50);						 		/* delay time more than 50ms */
    #endif
    #ifdef Stack_Z
	  while((GPIOB->IDR & Sensor_Z_NDRDY)){};      		/* DRDY信号为低时发送连续读数据指令，如果同步做得好，可以不要 */
    while((Sensor_Z->SR & SPI_I2S_FLAG_TXE) == 0);  	/* 等待发送寄存器为空 */
    Sensor_Z->DR = ADS1255_CMD_SELFOCAL;              /* Send bias and gain self-calibration instructions */
    #endif
	  Delay_us(50);						 		/* delay time more than 50ms */
}
/*******************************************************************************
    函数名：Self_GCal
    输  入: 
    输  出:
    功能说明:offset self-calibration
*/
void Self_GCal(void)
{
    #ifdef Stack_X
		while(GPIO_ReadInputDataBit(GPIOC,Sensor_X_NDRDY)){}; /* read data while DRDY is low */
    while(SPI_I2S_GetFlagStatus(Sensor_X, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(Sensor_X, ADS1255_CMD_SELFGCAL); 
    //Delay_us(50);							  	/* delay time more than 50ms	*/
    #endif	
    #ifdef Stack_Y
	  while(GPIO_ReadInputDataBit(GPIOB,Sensor_Y_NDRDY)){}; /* read data while DRDY is low */
    while(SPI_I2S_GetFlagStatus(Sensor_Y, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(Sensor_Y, ADS1255_CMD_SELFGCAL); 
    //Delay_us(50);							  	/* delay time more than 50ms	*/
    #endif	
    #ifdef Stack_Z 
	  while(GPIO_ReadInputDataBit(GPIOB,Sensor_Z_NDRDY)){}; /* read data while DRDY is low */
    while(SPI_I2S_GetFlagStatus(Sensor_Z, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(Sensor_Z, ADS1255_CMD_SELFGCAL); 
	  #endif	
		Delay_us(50);							  	/* delay time more than 50ms	*/
}

/*******************************************************************************
    函数名：Sys_OCal
    输  入: 
    输  出:
    功能说明:system offset calibration
*/
void Sys_OCal(void)
{
    #ifdef Stack_X
		while(GPIO_ReadInputDataBit(GPIOC,Sensor_X_NDRDY)){}; /* read data while DRDY is low */
    while(SPI_I2S_GetFlagStatus(Sensor_X, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(Sensor_X, ADS1255_CMD_SYSOCAL); 
    //Delay_us(100);							  	/* delay time more than 50ms	*/
    #endif	
    #ifdef Stack_Y
	  while(GPIO_ReadInputDataBit(GPIOB,Sensor_Y_NDRDY)){}; /* read data while DRDY is low */
    while(SPI_I2S_GetFlagStatus(Sensor_Y, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(Sensor_Y, ADS1255_CMD_SYSOCAL); 
    //Delay_us(100);							  	/* delay time more than 50ms	*/
    #endif	
    #ifdef Stack_Z 
	  while(GPIO_ReadInputDataBit(GPIOB,Sensor_Z_NDRDY)){}; /* read data while DRDY is low */
    while(SPI_I2S_GetFlagStatus(Sensor_Z, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(Sensor_Z, ADS1255_CMD_SYSOCAL); 
	  #endif	
		Delay_us(50);							  	/* delay time more than 50ms	*/
}

/*******************************************************************************
    函数名：Sys_GCal
    输  入: 
    输  出:
    功能说明:system gain calibration
*/
void Sys_GCal(void)
{
    #ifdef Stack_X
		while(GPIO_ReadInputDataBit(GPIOC,Sensor_X_NDRDY)){}; /* read data while DRDY is low */
    while(SPI_I2S_GetFlagStatus(Sensor_X, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(Sensor_X, ADS1255_CMD_SYSGCAL); 
    //Delay_us(50);							  	/* delay time more than 50ms	*/
    #endif	
    #ifdef Stack_Y
	  while(GPIO_ReadInputDataBit(GPIOB,Sensor_Y_NDRDY)){}; /* read data while DRDY is low */
    while(SPI_I2S_GetFlagStatus(Sensor_Y, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(Sensor_Y, ADS1255_CMD_SYSGCAL); 
    //Delay_us(50);							  	/* delay time more than 50ms	*/
    #endif	
    #ifdef Stack_Z 
	  while(GPIO_ReadInputDataBit(GPIOB,Sensor_Z_NDRDY)){}; /* read data while DRDY is low */
    while(SPI_I2S_GetFlagStatus(Sensor_Z, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(Sensor_Z, ADS1255_CMD_SYSGCAL); 
	  #endif	
		Delay_us(50);							  	/* delay time more than 50ms	*/
}


/*******************************************************************************
    函数名：SYN_CON
    输  入: 
    输  出:
    功能说明:Synchronize the AD conversion
*/
void SYN_CON(void)
{
	  #ifdef Stack_X
    while(SPI_I2S_GetFlagStatus(Sensor_X, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(Sensor_X, ADS1255_CMD_SYNC); 
    #endif	
    #ifdef Stack_Y
    while(SPI_I2S_GetFlagStatus(Sensor_Y, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(Sensor_Y, ADS1255_CMD_SYNC); 
    #endif	
    #ifdef Stack_Z 
    while(SPI_I2S_GetFlagStatus(Sensor_Z, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(Sensor_Z, ADS1255_CMD_SYNC); 
	  #endif	
		Delay_us(10);							  	/* delay time more than 3.125uS */
		#ifdef Stack_X
    while(SPI_I2S_GetFlagStatus(Sensor_X, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(Sensor_X, ADS1255_CMD_WAKEUP); 
    #endif	
    #ifdef Stack_Y
    while(SPI_I2S_GetFlagStatus(Sensor_Y, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(Sensor_Y, ADS1255_CMD_WAKEUP); 
    #endif	
    #ifdef Stack_Z 
    while(SPI_I2S_GetFlagStatus(Sensor_Z, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(Sensor_Z, ADS1255_CMD_WAKEUP); 
	  #endif	
}


/*******************************************************************************
    函数名：Standby_Mode
    输  入: 
    输  出:
    功能说明:Set Standby mode
*/
void Standby_Mode(void)
{
    #ifdef Stack_X
    while(SPI_I2S_GetFlagStatus(Sensor_X, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(Sensor_X, ADS1255_CMD_STANDBY); 
    #endif	
    #ifdef Stack_Y
    while(SPI_I2S_GetFlagStatus(Sensor_Y, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(Sensor_Y, ADS1255_CMD_STANDBY); 
    #endif	
    #ifdef Stack_Z 
    while(SPI_I2S_GetFlagStatus(Sensor_Z, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(Sensor_Z, ADS1255_CMD_STANDBY); 
		#endif
		Delay_us(50);
}


/*******************************************************************************
	???:AD_RD_Data_Continous
	?  ?: 
	?  ?:
	????:??SPI????????,???AD??
*/
  void AD_RD_Data_Continous(uint8_t *Data_X, uint8_t *Data_Y, uint8_t *Data_Z, s32 Con_Num)
{
	  #ifdef Stack_X
    uint32_t Loop_X = 0;		/* 避免在读取过程出现乘法运算，提高速度 */
	  #endif
	  #ifdef Stack_Y
    uint32_t Loop_Y = 0;
	  #endif
	  #ifdef Stack_Z
    uint32_t Loop_Z = 0;
	  #endif
    uint32_t preNum = 4; 
    //taskENTER_CRITICAL();
	 // vTaskSuspendAll();
    /***********安全起见在启动同步采集之前先确保各SPI数据寄存器为空***********/
	  #ifdef Stack_X
		while(SPI_I2S_GetFlagStatus(Sensor_X, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		#endif
	  #ifdef Stack_Y
		while(SPI_I2S_GetFlagStatus(Sensor_Y, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
	  #endif
	  #ifdef Stack_Z
		while(SPI_I2S_GetFlagStatus(Sensor_Z, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		#endif
    /*************************************************************************/
    /*----------------------------产生同步闸门信号--------------------------------*/
		GPIO_ResetBits(GPIOA , ADC_SYNC);
    Delay_us(3);  ///* low time must more than 0.52uS */
		GPIO_SetBits(GPIOA , ADC_SYNC);
    /* -----------为提高读取速度，采用空间换取时间的方法来提高存取速度------------*/
    /****************向ADC发送连续读取数据指令******************/
    #ifdef Stack_X 
		while(GPIO_ReadInputDataBit(GPIOC,Sensor_X_NDRDY)){}; /* read data while DRDY is low */
		SPI_I2S_SendData(Sensor_X, ADS1255_CMD_RDATAC); 
		#endif
		#ifdef Stack_Y
		while(GPIO_ReadInputDataBit(GPIOB,Sensor_Y_NDRDY)){}; /* read data while DRDY is low */
		SPI_I2S_SendData(Sensor_Y, ADS1255_CMD_RDATAC); 
		#endif
		#ifdef Stack_Z 
		while(GPIO_ReadInputDataBit(GPIOB,Sensor_Z_NDRDY)){}; /* read data while DRDY is low */ 
		SPI_I2S_SendData(Sensor_Z, ADS1255_CMD_RDATAC); 
    #endif
		#ifdef Stack_X
		while(SPI_I2S_GetFlagStatus(Sensor_X, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		#endif
		#ifdef Stack_Y
		while(SPI_I2S_GetFlagStatus(Sensor_Y, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		#endif
		#ifdef Stack_Z
		while(SPI_I2S_GetFlagStatus(Sensor_Z, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		#endif
    Delay_us(10);									/* 延时>6.5us */  
    /*************************进入循环读取过程***************************/
    while(preNum--)
    {  
			  #ifdef Stack_X
        while(GPIO_ReadInputDataBit(GPIOC,Sensor_X_NDRDY)){}; /* read data while DRDY is low */					
				SPI_I2S_SendData(Sensor_X, 0x00); 
				while (SPI_I2S_GetFlagStatus(Sensor_X, SPI_I2S_FLAG_RXNE) == RESET){} 
        Data_X[Loop_X++] = SPI_I2S_ReceiveData(Sensor_X); 	  /* read data have been received */
				//while(SPI_I2S_GetFlagStatus(Sensor_X, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
				SPI_I2S_SendData(Sensor_X, 0x00); 
				while (SPI_I2S_GetFlagStatus(Sensor_X, SPI_I2S_FLAG_RXNE) == RESET){} 
        Data_X[Loop_X++] = SPI_I2S_ReceiveData(Sensor_X); 	  /* read data have been received */
        //while(SPI_I2S_GetFlagStatus(Sensor_X, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
				SPI_I2S_SendData(Sensor_X, 0x00); 
				while (SPI_I2S_GetFlagStatus(Sensor_X, SPI_I2S_FLAG_RXNE) == RESET){} 
        Data_X[Loop_X++] = SPI_I2S_ReceiveData(Sensor_X); 	  /* read data have been received */
			  #endif
					
			  #ifdef Stack_Y
        while(GPIO_ReadInputDataBit(GPIOB,Sensor_Y_NDRDY)){}; /* read data while DRDY is low */					
				SPI_I2S_SendData(Sensor_Y, 0x00); 
				while (SPI_I2S_GetFlagStatus(Sensor_Y, SPI_I2S_FLAG_RXNE) == RESET){} 
        Data_Y[Loop_Y++] = SPI_I2S_ReceiveData(Sensor_Y); 	  /* read data have been received */
				//while(SPI_I2S_GetFlagStatus(Sensor_Y, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
				SPI_I2S_SendData(Sensor_Y, 0x00); 
				while (SPI_I2S_GetFlagStatus(Sensor_Y, SPI_I2S_FLAG_RXNE) == RESET){} 
        Data_Y[Loop_Y++] = SPI_I2S_ReceiveData(Sensor_Y); 	  /* read data have been received */
        //while(SPI_I2S_GetFlagStatus(Sensor_Y, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
				SPI_I2S_SendData(Sensor_Y, 0x00); 
				while (SPI_I2S_GetFlagStatus(Sensor_Y, SPI_I2S_FLAG_RXNE) == RESET){} 
        Data_Y[Loop_Y++] = SPI_I2S_ReceiveData(Sensor_Y); 	  /* read data have been received */
        #endif
					
			  #ifdef Stack_Z
        while(GPIO_ReadInputDataBit(GPIOB,Sensor_Z_NDRDY)){}; /* read data while DRDY is low */					
				SPI_I2S_SendData(Sensor_Z, 0x00); 
				while (SPI_I2S_GetFlagStatus(Sensor_Z, SPI_I2S_FLAG_RXNE) == RESET){} 
        Data_Z[Loop_Z++] = SPI_I2S_ReceiveData(Sensor_Z); 	  /* read data have been received */
				//while(SPI_I2S_GetFlagStatus(Sensor_Z, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
				SPI_I2S_SendData(Sensor_Z, 0x00); 
				while (SPI_I2S_GetFlagStatus(Sensor_Z, SPI_I2S_FLAG_RXNE) == RESET){} 
        Data_Z[Loop_Z++] = SPI_I2S_ReceiveData(Sensor_Z); 	  /* read data have been received */
        //while(SPI_I2S_GetFlagStatus(Sensor_Z, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
				SPI_I2S_SendData(Sensor_Z, 0x00); 
				while (SPI_I2S_GetFlagStatus(Sensor_Z, SPI_I2S_FLAG_RXNE) == RESET){} 
        Data_Z[Loop_Z++] = SPI_I2S_ReceiveData(Sensor_Z); 	  /* read data have been received */
        #endif
			  /******************************************************************/      
    }
		#ifdef Stack_X
    Loop_X = 0;					/* 避免在读取过程出现乘法运算，提高速度 */
		#endif
		#ifdef Stack_Y
    Loop_Y = 0;
		#endif
		#ifdef Stack_Z
    Loop_Z = 0;
		#endif
    /*************************进入循环读取过程***************************/
    while(Con_Num--)
    {  
        #ifdef Stack_X
        while(GPIO_ReadInputDataBit(GPIOC,Sensor_X_NDRDY)){}; /* read data while DRDY is low */					
				SPI_I2S_SendData(Sensor_X, 0x00); 
				while (SPI_I2S_GetFlagStatus(Sensor_X, SPI_I2S_FLAG_RXNE) == RESET){} 
        Data_X[Loop_X++] = SPI_I2S_ReceiveData(Sensor_X); 	  /* read data have been received */
				//while(SPI_I2S_GetFlagStatus(Sensor_X, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
				SPI_I2S_SendData(Sensor_X, 0x00); 
				while (SPI_I2S_GetFlagStatus(Sensor_X, SPI_I2S_FLAG_RXNE) == RESET){} 
        Data_X[Loop_X++] = SPI_I2S_ReceiveData(Sensor_X); 	  /* read data have been received */
        //while(SPI_I2S_GetFlagStatus(Sensor_X, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
				SPI_I2S_SendData(Sensor_X, 0x00); 
				while (SPI_I2S_GetFlagStatus(Sensor_X, SPI_I2S_FLAG_RXNE) == RESET){} 
        Data_X[Loop_X++] = SPI_I2S_ReceiveData(Sensor_X); 	  /* read data have been received */
			  #endif
					
			  #ifdef Stack_Y
        while(GPIO_ReadInputDataBit(GPIOB,Sensor_Y_NDRDY)){}; /* read data while DRDY is low */					
				SPI_I2S_SendData(Sensor_Y, 0x00); 
				while (SPI_I2S_GetFlagStatus(Sensor_Y, SPI_I2S_FLAG_RXNE) == RESET){} 
        Data_Y[Loop_Y++] = SPI_I2S_ReceiveData(Sensor_Y); 	  /* read data have been received */
				//while(SPI_I2S_GetFlagStatus(Sensor_Y, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
				SPI_I2S_SendData(Sensor_Y, 0x00); 
				while (SPI_I2S_GetFlagStatus(Sensor_Y, SPI_I2S_FLAG_RXNE) == RESET){} 
        Data_Y[Loop_Y++] = SPI_I2S_ReceiveData(Sensor_Y); 	  /* read data have been received */
        //while(SPI_I2S_GetFlagStatus(Sensor_Y, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
				SPI_I2S_SendData(Sensor_Y, 0x00); 
				while (SPI_I2S_GetFlagStatus(Sensor_Y, SPI_I2S_FLAG_RXNE) == RESET){} 
        Data_Y[Loop_Y++] = SPI_I2S_ReceiveData(Sensor_Y); 	  /* read data have been received */
        #endif
					
			  #ifdef Stack_Z
        while(GPIO_ReadInputDataBit(GPIOB,Sensor_Z_NDRDY)){}; /* read data while DRDY is low */					
				SPI_I2S_SendData(Sensor_Z, 0x00); 
				while (SPI_I2S_GetFlagStatus(Sensor_Z, SPI_I2S_FLAG_RXNE) == RESET){} 
        Data_Z[Loop_Z++] = SPI_I2S_ReceiveData(Sensor_Z); 	  /* read data have been received */
				//while(SPI_I2S_GetFlagStatus(Sensor_Z, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
				SPI_I2S_SendData(Sensor_Z, 0x00); 
				while (SPI_I2S_GetFlagStatus(Sensor_Z, SPI_I2S_FLAG_RXNE) == RESET){} 
        Data_Z[Loop_Z++] = SPI_I2S_ReceiveData(Sensor_Z); 	  /* read data have been received */
        //while(SPI_I2S_GetFlagStatus(Sensor_Z, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
				SPI_I2S_SendData(Sensor_Z, 0x00); 
				while (SPI_I2S_GetFlagStatus(Sensor_Z, SPI_I2S_FLAG_RXNE) == RESET){} 
        Data_Z[Loop_Z++] = SPI_I2S_ReceiveData(Sensor_Z); 	  /* read data have been received */
        #endif
			  /******************************************************************/
    }
    Stop_RDC();    /* exit continuous read mode */
   // xTaskResumeAll();
		//taskEXIT_CRITICAL();
}

/*******************************************************************************
    函数名：ADS1255SetGain
    输  入: 
    输  出:
    功能说明:Separate gain setting
*/
void ADS1255SetGain(uint8_t GainCode)
{
	 #ifdef Stack_X
    AD_WR_Reg(Sensor_X, ADS1255_ADCON, GainCode);
    #endif
	  #ifdef Stack_Y
	  AD_WR_Reg(Sensor_Y, ADS1255_ADCON, GainCode);
    #endif
	  #ifdef Stack_Z
	  AD_WR_Reg(Sensor_Z, ADS1255_ADCON, GainCode);
    #endif
	  Delay_us(100);  
	  //Self_Cal();          /* self Gain and offset offset correction	*/
    //Sys_OCal();          /* System offset correction */
	  //Sys_HCal();          /* System gain correction */
}

/*******************************************************************************
    函数名：ADS1255SetRate
    输  入: 
    输  出:
    功能说明:Separate sampling rate choice
*/
void ADS1255SetRate(uint8_t Rate)
{
	  #ifdef Stack_X
    AD_WR_Reg(Sensor_X, ADS1255_DRATE, Rate);
    #endif
	  #ifdef Stack_Y
	  AD_WR_Reg(Sensor_Y, ADS1255_DRATE, Rate);
    #endif
	  #ifdef Stack_Z
	  AD_WR_Reg(Sensor_Z, ADS1255_DRATE, Rate);
    #endif
	  Delay_us(100);
    //Self_Cal();          /* self Gain and offset offset correction	*/
    //Sys_OCal();          /* System offset correction */
	  //Sys_HCal();          /* System gain correction */
}
/*******************************************************************************
    函数名：ADS1255SetMux
    输  入: 
    输  出:
    功能说明:Separate Mux choice
*/
void ADS1255SetMux(uint8_t Mux)
{
	 #ifdef Stack_X
    AD_WR_Reg(Sensor_X, ADS1255_MUX, Mux);
	  #endif
	  #ifdef Stack_Y
    AD_WR_Reg(Sensor_Y, ADS1255_MUX, Mux);
	  #endif
	  #ifdef Stack_Z
    AD_WR_Reg(Sensor_Z, ADS1255_MUX, Mux);
	  #endif
	  Delay_us(100);
    
    //Self_Cal();          /* self Gain and offset offset correction	*/
    //Sys_OCal();          /* System offset correction */
	  //Sys_HCal();          /* System gain correction */
}

/*******************************************************************************
    函数名：ADS1255SetMux_sigle
    输  入: 
    输  出:
    功能说明:Separate Mux choice
*/
void ADS1255SetMux_sigle(SPI_TypeDef* SPIx,uint8_t Mux)
{
    AD_WR_Reg(SPIx, ADS1255_MUX, Mux);
	  //Delay_us(500);
    
    //Self_Cal();          /* self Gain and offset offset correction	*/
    //Sys_OCal();          /* System offset correction */
	  //Sys_HCal();          /* System gain correction */
}
/*******************************************************************************
    函数名：ADS1255_RESET
    输  入: 
    输  出:
    功能说明:
*/
void ADS1255_RESET(void)
{
	  #ifdef Stack_X
		while(GPIO_ReadInputDataBit(GPIOC,Sensor_X_NDRDY)){}; /* read data while DRDY is low */		
		while(SPI_I2S_GetFlagStatus(Sensor_X, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(Sensor_X, ADS1255_CMD_RESET); 
    #endif	
			
    #ifdef Stack_Y
		while(GPIO_ReadInputDataBit(GPIOB,Sensor_Y_NDRDY)){}; /* read data while DRDY is low */		
		while(SPI_I2S_GetFlagStatus(Sensor_Y, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(Sensor_Y, ADS1255_CMD_RESET); 
    #endif	
			
    #ifdef Stack_Z 
		while(GPIO_ReadInputDataBit(GPIOB,Sensor_Z_NDRDY)){}; /* read data while DRDY is low */		
		while(SPI_I2S_GetFlagStatus(Sensor_Z, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		SPI_I2S_SendData(Sensor_Z, ADS1255_CMD_RESET); 
	  #endif	
		Delay_us(1000);
}
/*=================================================================================*/
/*******************************************************************************
    函数名：ad_modules_init
    输  入: 
    输  出:
    功能说明:AD初始化
*/
void ad_modules_init(void)
{
    ADC_SPI_Port_Configuration();	
    ADC_SPI_Init();
    AllSPI_Enable();
	  ADC_Parameter_Set();
    Delay_us(2500);
	  ADS1255_RESET();
	  Delay_us(500);
    ADC_Parameter_Config();	/* Complete all SPI initialization configuration and self-calibration */ 
    //置通道等相关位置
    momal_sample_gpio_init();
}


/*******************************************************************************
    函数名：start_pre_sample
    输  入: 
    输  出:
    功能说明:启动预采集
*/
void start_pre_sample(void)
{
    //pre_sample->currentPointer = 0;
    //pre_sample->preSamplingSwitch = ENABLE;
    //打开AD采集
	  ADS1255_RESET();
	  ADC_Parameter_Config();
    Delay_us(100);//延时，等待信号源稳定
    /***********安全起见在启动同步采集之前先确保各SPI数据寄存器为空***********/
	  #ifdef Stack_X
		while(SPI_I2S_GetFlagStatus(Sensor_X, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		#endif
	  #ifdef Stack_Y
		while(SPI_I2S_GetFlagStatus(Sensor_Y, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
	  #endif
	  #ifdef Stack_Z
		while(SPI_I2S_GetFlagStatus(Sensor_Z, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		#endif
	  /*************************************************************************/
    /*----------------------------产生同步闸门信号--------------------------------*/
		GPIO_ResetBits(GPIOA , ADC_SYNC);
    Delay_us(3);  ///* low time must more than 0.52uS */
		GPIO_SetBits(GPIOA , ADC_SYNC);
    /* -----------为提高读取速度，采用空间换取时间的方法来提高存取速度------------*/
    /****************向ADC发送连续读取数据指令******************/
     #ifdef Stack_X
		while(GPIO_ReadInputDataBit(GPIOC,Sensor_X_NDRDY)){}; /* read data while DRDY is low */
		SPI_I2S_SendData(Sensor_X, ADS1255_CMD_RDATAC); 
		#endif
		#ifdef Stack_Y
		while(GPIO_ReadInputDataBit(GPIOB,Sensor_Y_NDRDY)){}; /* read data while DRDY is low */
		SPI_I2S_SendData(Sensor_Y, ADS1255_CMD_RDATAC); 
		#endif
		#ifdef Stack_Z
		while(GPIO_ReadInputDataBit(GPIOB,Sensor_Z_NDRDY)){}; /* read data while DRDY is low */  
		SPI_I2S_SendData(Sensor_Z, ADS1255_CMD_RDATAC); 
    #endif
		
		#ifdef Stack_X
		while(SPI_I2S_GetFlagStatus(Sensor_X, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		#endif
		#ifdef Stack_Y
		while(SPI_I2S_GetFlagStatus(Sensor_Y, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		#endif
		#ifdef Stack_Z
		while(SPI_I2S_GetFlagStatus(Sensor_Z, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
		#endif
    Delay_us(100);										/* 延时>6.5us */
}

void AD_Work(void)
{
	  remainNumPoints=2048-sets.preSamplingTime_points;
		//SDA_H;
	  start_pre_sample();
	  while(1)
		{
        //
			  #ifdef Stack_X
				while(GPIO_ReadInputDataBit(GPIOC,Sensor_X_NDRDY)){}; /* read data while DRDY is low */					
				SPI_I2S_SendData(Sensor_X, 0x00); 
				while (SPI_I2S_GetFlagStatus(Sensor_X, SPI_I2S_FLAG_RXNE) == RESET){} 
				sample_data[(pre_sample.currentPointer)*3] = SPI_I2S_ReceiveData(Sensor_X); 	  /* read data have been received */
				//while(SPI_I2S_GetFlagStatus(Sensor_X, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
				SPI_I2S_SendData(Sensor_X, 0x00); 
				while (SPI_I2S_GetFlagStatus(Sensor_X, SPI_I2S_FLAG_RXNE) == RESET){} 
				sample_data[(pre_sample.currentPointer)*3+1] = SPI_I2S_ReceiveData(Sensor_X); 	  /* read data have been received */
				//while(SPI_I2S_GetFlagStatus(Sensor_X, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
				SPI_I2S_SendData(Sensor_X, 0x00); 
				while (SPI_I2S_GetFlagStatus(Sensor_X, SPI_I2S_FLAG_RXNE) == RESET){} 
				sample_data[(pre_sample.currentPointer)*3+2] = SPI_I2S_ReceiveData(Sensor_X); 	  /* read data have been received */
				#endif
					
				#ifdef Stack_Y
				//while(GPIO_ReadInputDataBit(GPIOB,Sensor_Y_NDRDY)){}; /* read data while DRDY is low */					
				SPI_I2S_SendData(Sensor_Y, 0x00); 
				while (SPI_I2S_GetFlagStatus(Sensor_Y, SPI_I2S_FLAG_RXNE) == RESET){} 
				sample_data[BaseAdress_YData+(pre_sample.currentPointer)*3] = SPI_I2S_ReceiveData(Sensor_Y); 	  /* read data have been received */
				//while(SPI_I2S_GetFlagStatus(Sensor_Y, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
				SPI_I2S_SendData(Sensor_Y, 0x00); 
				while (SPI_I2S_GetFlagStatus(Sensor_Y, SPI_I2S_FLAG_RXNE) == RESET){} 
				sample_data[BaseAdress_YData+(pre_sample.currentPointer)*3+1] = SPI_I2S_ReceiveData(Sensor_Y); 	  /* read data have been received */
				//while(SPI_I2S_GetFlagStatus(Sensor_Y, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
				SPI_I2S_SendData(Sensor_Y, 0x00); 
				while (SPI_I2S_GetFlagStatus(Sensor_Y, SPI_I2S_FLAG_RXNE) == RESET){} 
				sample_data[BaseAdress_YData+(pre_sample.currentPointer)*3+2] = SPI_I2S_ReceiveData(Sensor_Y); 	  /* read data have been received */
				#endif
					
				#ifdef Stack_Z
				//while(GPIO_ReadInputDataBit(GPIOB,Sensor_Z_NDRDY)){}; /* read data while DRDY is low */					
				SPI_I2S_SendData(Sensor_Z, 0x00); 
				while (SPI_I2S_GetFlagStatus(Sensor_Z, SPI_I2S_FLAG_RXNE) == RESET){} 
				sample_data[BaseAdress_ZData+(pre_sample.currentPointer)*3] = SPI_I2S_ReceiveData(Sensor_Z); 	  /* read data have been received */
				//while(SPI_I2S_GetFlagStatus(Sensor_Z, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
				SPI_I2S_SendData(Sensor_Z, 0x00); 
				while (SPI_I2S_GetFlagStatus(Sensor_Z, SPI_I2S_FLAG_RXNE) == RESET){} 
				sample_data[BaseAdress_ZData+(pre_sample.currentPointer)*3+1] = SPI_I2S_ReceiveData(Sensor_Z); 	  /* read data have been received */
				//while(SPI_I2S_GetFlagStatus(Sensor_Z, SPI_I2S_FLAG_TXE) == RESET){};  		/* 等待发送寄存器为空 */
				SPI_I2S_SendData(Sensor_Z, 0x00); 
				while (SPI_I2S_GetFlagStatus(Sensor_Z, SPI_I2S_FLAG_RXNE) == RESET){} 
				sample_data[BaseAdress_ZData+(pre_sample.currentPointer)*3+2] = SPI_I2S_ReceiveData(Sensor_Z); 	  /* read data have been received */
				#endif
			  pre_sample.currentPointer++;
			  if(pre_sample.currentPointer>=2048)
				{
					  pre_sample.currentPointer=0;
				} 
				if(pre_sample.syncTriggerStatus==1)
				{
					  remainNumPoints--;
					  if(remainNumPoints<=1)
						{
							  //关闭ADC
								//SDA_L;
							  return;
						}
				}
		}
}

void ADC_Parameter_Config_RFLASH(void)
{
		u32 R_DATA[6];
		STMFLASH_Read(FLASH_SAVE_PARAMETER_ADDR,(u32*)R_DATA,6);
		if( R_DATA[0]!=0x55  )//简单判断
		{
				/*AD默认配置参数*/ 
        sets.numOfPoint_level=0x01;		
        sets.sampleFrequency_level=0x04;					
				sets.preSamplingTime_points=3;//3 个采样点
			  sets.delayTime_ms=0x00;
				sets.headGain=0x01;
		}
	else 
	{	
				sets.numOfPoint_level=(uint8_t)R_DATA[1];		
        sets.sampleFrequency_level=(uint8_t)R_DATA[2];					
				sets.preSamplingTime_points=(uint8_t)R_DATA[3];
			  sets.delayTime_ms=(uint8_t)R_DATA[4];
				sets.headGain=(uint8_t)R_DATA[5];
	}
}

void ADC_Parameter_Save_WFLASH(void)
{
	  u32 W_DATA[8];
	  W_DATA[0] = 0x55;
		W_DATA[1] = (u32)Local_node_num;
	  W_DATA[2]=0x55;
	  W_DATA[3]=(u32)sets.numOfPoint_level;
	  W_DATA[4]=(u32)sets.sampleFrequency_level;
	  W_DATA[5]=(u32)sets.preSamplingTime_points;
	  W_DATA[6]=(u32)sets.delayTime_ms;
	  W_DATA[7]=(u32)sets.headGain;
		STMFLASH_Write(FLASH_SAVE_NODENUM_ADDR,(u32*)(W_DATA),8);//写入8个字		
}

/*******************************************************************************
    函数名：AD_TEST
    输  入: 
    输  出:
    功能说明:AD测试函数
*/
void AD_TEST(void)
{
//     uint8_t DataX[64*3];
//     uint8_t DataY[64*3];
//     uint8_t DataZ[64*3];
    //uint32_t SampleQuantity = 64;
    while(1)
    {
        //AD_Contine_Convertion(DataX, DataY, DataZ, SampleQuantity);
			  //AD_RD_Data_Continous(DataX, DataY, DataZ, SampleQuantity);
			  //Filter_DC(&DataZ[0],64);
			  //AD_RD_MultiReg(Sensor_X,(uint8_t)ADS1255_STATUS,11,statusA);
			  //rate=AD_RD_Reg(Sensor_X,ADS1255_DRATE);
 			  //AD_RD_MultiReg(Sensor_Y,(uint8_t)ADS1255_STATUS,11,statusA);
 			  //rate=AD_RD_Reg(Sensor_Y,ADS1255_DRATE);
    }
//     //return;
// 		uint8_t statusA[11];
	  //uint8_t  rate;
		//rate=AD_RD_Reg(Sensor_Z,ADS1255_DRATE);
// 		AD_RD_MultiReg(Sensor_Z,(uint8_t)ADS1255_STATUS,11,statusA);
}


void ADC_Parameter_Set(void)
{
    switch(sets.sampleFrequency_level)
    {
    case 0x01:
        ADS1255_DRATEVALUE = ADS1256_DRATE_15000SPS;
        break;
    case 0x02:
        ADS1255_DRATEVALUE = ADS1256_DRATE_7500SPS;
        break;
    case 0x03:
        ADS1255_DRATEVALUE = ADS1256_DRATE_3750SPS;
        break;
    case 0x04:
        ADS1255_DRATEVALUE = ADS1256_DRATE_2000SPS;
        break;
    case 0x05:
        ADS1255_DRATEVALUE = ADS1256_DRATE_1000SPS;
        break;
    case 0x06:
        ADS1255_DRATEVALUE = ADS1256_DRATE_500SPS;
        break;
    case 0x07:
        ADS1255_DRATEVALUE = ADS1256_DRATE_100SPS;
        break;
    default :
        ADS1255_DRATEVALUE = ADS1256_DRATE_2000SPS;
        break;
    }
		switch(sets.headGain)
    {
			case 0x01:
        ADS1255_GAIN = ADS1255_GAIN_1;
        break;
			case 0x02:
        ADS1255_GAIN = ADS1255_GAIN_2;
        break;
			case 0x03:
        ADS1255_GAIN = ADS1255_GAIN_4;
        break;
			case 0x04:
        ADS1255_GAIN = ADS1255_GAIN_8;
        break;
			case 0x05:
        ADS1255_GAIN = ADS1255_GAIN_16;
        break;
			case 0x06:
        ADS1255_GAIN = ADS1255_GAIN_32;
        break;
			case 0x07:
        ADS1255_GAIN = ADS1255_GAIN_64;
        break;
			default :
        ADS1255_GAIN = ADS1255_GAIN_1;
        break;
		}
}


uint8_t ADC_LoadDataserver(uint8_t data_load_style)
{
	uint8_t stack_num = 0;
	uint8_t pack_num = 0;
	//----------------------------------
	int LoadBaseAddress = 0;
	//----------------------------------
//	uint8_t i;
	*PPP_send_Dest = HOST_ADDRESS;
	*PPP_send_sour = Local_node_num;
	*PPP_data_length = 100;
	switch (data_load_style)
	{
		case LOAD_DATA_REQUEST:
			//组装预采样数据
		  //--------------------------------------------------------------------------
		  if(pre_sample.syncSamplingPosition-sets.preSamplingTime_points>=0)
			{
				LoadBaseAddress=pre_sample.syncSamplingPosition-sets.preSamplingTime_points;
			}
			else
			{
				LoadBaseAddress=pre_sample.syncSamplingPosition+Default_SampleNum-sets.preSamplingTime_points;
			}
			//---------------------------------------------------------------------------
		  *PPP_send_type = LOAD_DATA_REPLY;
			break;
		case LOAD_NOISE_REQUEST:
			background_noise();//
		  *PPP_send_type = LOAD_NOISE_REPLY;
			break;
		case TEST_GAIN_REQUEST:
			//test_gain();//
		  test_gain_Pulsewave();//2017-09-12 +
		  *PPP_send_type = TEST_GAIN_REPLY;
			break;
		case TEST_WITHIN_NOISE_REQUEST:
			test_within_noise();//
		  *PPP_send_type = TEST_WITHIN_NOISE_REPLY;
			break;
		case TEST_IMPULSE_RESPONSE_REQUEST:
			test_impulse_response();//
		  *PPP_send_type = TEST_IMPULSE_RESPONSE_REPLY;
			break;
		case TEST_CONFORMANCE_REQUEST:
			test_conformance();//
		  *PPP_send_type = TEST_CONFORMANCE_REPLY;
			break;
		case TSET_CROSSTALK_REQUEST:
			test_crosstalk();//
		  *PPP_send_type = TEST_CROSSTALK_REPLY;
			break;
		case TEST_COMMON_MODE_REQUEST:
			test_common_mode();//
		  *PPP_send_type = TEST_COMMON_MODE_REPLY;
			break;
		case TEST_DISTORTION_REQUEST:
			test_distortion();//
		  *PPP_send_type = TEST_DISTORTION_REPLY;
			break;
		case TEST_RESISTANCE_REQUEST:
/*			
			AD重新读取FLASH中的参数，并配置
*/
			ADC_Parameter_Config_RFLASH();
			ADC_Parameter_Set();
			ADC_Parameter_Config();	/* Complete all SPI initialization configuration and self-calibration */ 
			
			test_res();
			*PPP_send_type= TEST_RESISTANCE_REPLY;
			break;
		//添加传感器检测模块
		case TEST_WITHIN_NOISE_SENSOR_REQUEST:
			sensor_test_within_noise();
			*PPP_send_type= TEST_WITHIN_NOISE_SENSOR_REPLY;
			break;
		case TEST_DISTORTION_SENSOR_REQUEST:
			sensor_test_distortion();
			*PPP_send_type= TEST_DISTORTION_SENSOR_REPLY;
			break;
		case TEST_COMMON_MODE_SENSOR_REQUEST:
			sensor_test_common_mode();
			*PPP_send_type= TEST_COMMON_MODE_SENSOR_REPLY;
			break;
		case TEST_IMPULSE_RESPONSE_SENSOR_REQUEST:
			sensor_test_impulse_response();
			*PPP_send_type= TEST_IMPULSE_RESPONSE_SENSOR_REPLY;
			break;
		case TEST_TILT_SENSOR_REQUEST:
			sensor_test_tilt();
			*PPP_send_type= TEST_TILT_SENSOR_REPLY;
			break;
		case TEST_GRAVITY_SENSOR_REQUEST:
			sensor_test_gravity();
			*PPP_send_type= TEST_GRAVITY_SENSOR_REPLY;
			break;		
		//添加传感器检测模块结束
		default:
			break;
	}	
	for(stack_num=0;stack_num<3;stack_num++)//打包三个通道
	{
		for(pack_num=0;pack_num<64;pack_num++)//将每一道数据打包成64包
		{
			PPP_send_data[0]=Local_node_num;
			PPP_send_data[1]=stack_num;
			PPP_send_data[2]=pack_num;
			PPP_send_data[3]=96;
			//---------------------------------------------------------------------------------------------------------
			if(LoadBaseAddress+pack_num*32<=2016)//发送63包
			{
				memcpy(&PPP_send_data[4],&sample_data[(stack_num*2048+LoadBaseAddress+pack_num*32)*3],32*3);
			}
			else if(LoadBaseAddress+pack_num*32<2048)
			{
				memcpy(&PPP_send_data[4],&sample_data[(stack_num*2048+LoadBaseAddress+pack_num*32)*3],(2048-LoadBaseAddress-pack_num*32)*3);
				memcpy(&PPP_send_data[4+(2048-LoadBaseAddress-pack_num*32)*3],&sample_data[stack_num*2048*3],(32-(2048-LoadBaseAddress-pack_num*32))*3);
			}
			else //(LoadBaseAddress+pack_num*32>=2048)
			{
				memcpy(&PPP_send_data[4],&sample_data[(stack_num*2048+LoadBaseAddress+pack_num*32-2048)*3],32*3);
			}
			//---------------------------------------------------------------------------------------------------------
			//memcpy(&PPP_send_data[4],&sample_data[stack_num*2048*3+pack_num*32*3],32*3);
			ppp_send_data(104);
			//vTaskDelay(1);
		}
	}
	return 0;
}

/**
  * @brief  ADCPre_thread
  * @param arg: pointer on argument(not used here) 
  * @retval None
  */
static void ADCPre_thread(void *arg)
{
	uint8_t adc_Prestart;
	uint8_t adc_finish;
	portBASE_TYPE xStatus;
	while(1)
	{
		xStatus = xQueueReceive( xQueue_ADPreStart, &adc_Prestart, portMAX_DELAY );//INCLUDE_vTaskSuspend?
		if( xStatus == pdPASS )
		{
			  //SDA_H;
			  //添加初始化或复位ADC工作
			  pre_sample.currentPointer=0;
				//pre_sample->preSamplingSwitch=0;
				pre_sample.syncSamplingPosition=0;
				pre_sample.syncTriggerStatus=0;
			  AD_Work();
			  //采集2048个点
			  xQueueSendToBack(xQueue_ADFinish, &adc_finish, 0 );	
		}
	}
}
/**
  * @brief  ADC_LoadDatathread
  * @param arg: pointer on argument(not used here) 
  * @retval None
  */
static void ADC_LoadDatathread(void *arg)
{
	uint8_t data_load_style;
	portBASE_TYPE xStatus;
  while (1) 
  {
		xStatus = xQueueReceive( xQueue_LoadData, &data_load_style, portMAX_DELAY );//INCLUDE_vTaskSuspend?
    if( xStatus == pdPASS )
		{
			ADC_LoadDataserver(data_load_style);	
		}
  }
}
//    函数名：ADC_init
//    输  入: 
//    输  出:
//    功能说明：
//*/
void ADC_init()
{
	int i;
	if(Sample_Data_init()==1)
	{
		while(1);//申请内存失败
	}
	for(i=0;i<2048*3*3;i++) 
	{
		sample_data[i]=150;
	}
	pre_sample.currentPointer=0;
	pre_sample.preSamplingSwitch=0;
	pre_sample.syncSamplingPosition=0;
	pre_sample.syncTriggerStatus=0;
	vTaskDelay(500);
	ADC_Parameter_Config_RFLASH();
	ad_modules_init();
	if( xQueue_ADPreStart != NULL )
	{
		xTaskCreate(ADCPre_thread, (int8_t *) "ADCPre", configMINIMAL_STACK_SIZE * 5, NULL,ADCPRE_THREAD_PRIO, NULL);
	}
	
	if( xQueue_LoadData != NULL )
	{
		xTaskCreate(ADC_LoadDatathread, (int8_t *) "LoadData", configMINIMAL_STACK_SIZE * 10, NULL,ADCLOAD_THREAD_PRIO, NULL);
	}
}



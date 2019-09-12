/*
  
* All rights reserved.
*
* File Name          : ADS1255.h
* Description        : ADS1255读写控制及配置相关函数
*
* Version            : V1.0
* Author             : liu Yong
* Date               : 7/7/2012
*
* OriginalVersion    : V1.0
* Original Author    : liu Yong
* Date               : 7/7/2012
*/
#ifndef __ADS1255_H__
#define __ADS1255_H__

#include "stm32f4xx.h"
/*===============================全局外部变量定义======================================*/
extern __IO uint8_t ADS1255_GAIN;				 	/* ADC增益控制变量 */
extern __IO uint8_t ADS1255_DRATEVALUE; 			/* ADC采样率控制变量 */
extern uint8_t *sample_data;
/*=================================================================================*/

#define Default_SampleNum 2048

#define BaseAdress_XData 0
#define BaseAdress_YData Default_SampleNum*3
#define BaseAdress_ZData Default_SampleNum*3*2


////单路测试宏定义
#define Stack_X 
#define Stack_Y 
#define Stack_Z 

/*=========================端口定义=========================*/
#define  ADC_SYNC			GPIO_Pin_8	 //PA8
#define  Sensor_X     SPI1	 	 //PA
#define	 Sensor_Y			SPI2     //PB
#define  Sensor_Z			SPI3	 	 //PC

#define  Sensor_X_SCLK    	GPIO_Pin_5   //PA5
#define  Sensor_X_DOUT    	GPIO_Pin_6   //PA6
#define  Sensor_X_DIN    	GPIO_Pin_7	 //PA7
#define  Sensor_X_NDRDY   	GPIO_Pin_4	 //PC4

#define  Sensor_Y_SCLK    	GPIO_Pin_13  //PB13
#define  Sensor_Y_DOUT    	GPIO_Pin_14  //PB14
#define  Sensor_Y_DIN    	GPIO_Pin_15	 //PB15
#define  Sensor_Y_NDRDY   	GPIO_Pin_12	 //PB12

#define  Sensor_Z_SCLK    	GPIO_Pin_10  //PC10
#define  Sensor_Z_DOUT    	GPIO_Pin_11  //PC11
#define  Sensor_Z_DIN    	GPIO_Pin_12	 //PC12
#define  Sensor_Z_NDRDY   	GPIO_Pin_5	 //PB5

#define  Sensor_Reset   	GPIO_Pin_7	 //PC7

#define  Sensor_Reset_ON      GPIO_SetBits(GPIOC , GPIO_Pin_7)
#define  Sensor_Reset_OFF     GPIO_ResetBits(GPIOC , GPIO_Pin_7)


#define  PC8_SENSOR_CHECK  GPIO_Pin_8   //PC8
#define  SENSOR_CHECK_ON      GPIO_SetBits(GPIOC , GPIO_Pin_8)
#define  SENSOR_CHECK_OFF     GPIO_ResetBits(GPIOC , GPIO_Pin_8)

/*=========================操作命令=========================*/
#define ADS1255_CMD_WAKEUP   0x00	//Completes SYNC and Exits Standby Mode
#define ADS1255_CMD_RDATA    0x01	//Read Data
#define ADS1255_CMD_RDATAC   0x03	//Read Data Continuously
#define ADS1255_CMD_SDATAC   0x0f	//Stop Read Data Continuously
#define ADS1255_CMD_RREG     0x10	//Read from REG rrr
#define ADS1255_CMD_WREG     0x50	//Write to REG rrr
#define ADS1255_CMD_SELFCAL  0xf0	//Offset and Gain Self-Calibration
#define ADS1255_CMD_SELFOCAL 0xf1	//Offset Self-Calibration
#define ADS1255_CMD_SELFGCAL 0xf2	//Gain Self-Calibration
#define ADS1255_CMD_SYSOCAL  0xf3	//System Offset Calibration
#define ADS1255_CMD_SYSGCAL  0xf4	//System Gain Calibration
#define ADS1255_CMD_SYNC     0xfc	//Synchronize the A/D Conversion
#define ADS1255_CMD_STANDBY  0xfd	//Begin Standby Mode
#define ADS1255_CMD_RESET    0xfe	//Reset to Power-Up Values

/*=================ADS1255内部寄存器地址映射====================*/
#define ADS1255_STATUS       0x00  	//STATUS REGISTER (ADDRESS 00h)
#define ADS1255_MUX          0x01  	//Input Multiplexer Control Register (Address 01h)
#define ADS1255_ADCON        0x02  	//A/D Control Register (Address 02h)
#define ADS1255_DRATE        0x03  	//A/D Data Rate (Address 03h)
#define ADS1255_IO           0x04  	//GPIO Control Register (Address 04H)
#define ADS1255_OFC0         0x05  	//Offset Calibration Byte 0, least significant byte (Address 05h)
#define ADS1255_OFC1         0x06  	//Offset Calibration Byte 1 (Address 06h)
#define ADS1255_OFC2         0x07  	//Offset Calibration Byte 2, most significant byte (Address 07h)
#define ADS1255_FSC0         0x08  	//Full-scale Calibration Byte 0, least significant byte (Address 08h)
#define ADS1255_FSC1         0x09  	//Full-scale Calibration Byte 1 (Address 09h)
#define ADS1255_FSC2         0x0A	//Full-scale Calibration Byte 2, most significant byte (Address 0Ah)

/*=====================ADS1255内置放大器增益宏=========================*/
#define ADS1255_GAIN_1      0x00
#define ADS1255_GAIN_2      0x01
#define ADS1255_GAIN_4      0x02
#define ADS1255_GAIN_8      0x03
#define ADS1255_GAIN_16     0x04
#define ADS1255_GAIN_32     0x05
#define ADS1255_GAIN_64     0x06
//#define ADS1256_GAIN_64     0x07

/*=========================ADC采样率配置宏=============================*/
#define ADS1256_DRATE_30000SPS  0xF0  //default
#define ADS1256_DRATE_15000SPS  0xE0
#define ADS1256_DRATE_7500SPS   0xD0
#define ADS1256_DRATE_3750SPS   0xC0
#define ADS1256_DRATE_2000SPS   0xB0
#define ADS1256_DRATE_1000SPS   0xA1
#define ADS1256_DRATE_500SPS    0x92
#define ADS1256_DRATE_100SPS    0x82
#define ADS1256_DRATE_60SPS     0x72
#define ADS1256_DRATE_50SPS     0x63
#define ADS1256_DRATE_30SPS     0x53
#define ADS1256_DRATE_25SPS     0x43
#define ADS1256_DRATE_15SPS     0x33
#define ADS1256_DRATE_10SPS     0x23
#define ADS1256_DRATE_5SPS      0x13
#define ADS1256_DRATE_2SPS      0x03

/*=========================ADC通道配置宏=============================*/
#define ADS1256_CHANNEL_DIFFERENTIAL_0P1N  0x01  //default
#define ADS1256_CHANNEL_SINGLE_5P  0x58  //
#define ADS1256_CHANNEL_SINGLE_7P  0x78  //
#define ADS1256_CHANNEL_SINGLE_0P  0x08  //

/*===========================ADC硬件同步==========================*/
#define ADC_SYNC_DEFI_STATE GPIO_SetBits(GPIOA , ADC_SYNC)						 //默认状态同步端口高电平

/*===========================ADC内部操作函数==========================*/
void ADC_SPI_Port_Configuration(void);
void ADC_SPI_Init(void);
void AllSPI_Enable(void);
void AllSPI_Disenable(void);
uint8_t AD_RD_Reg(SPI_TypeDef* SPIx, uint8_t reg);
void AD_WR_Reg(SPI_TypeDef* SPIx, uint8_t reg, uint8_t wdata);
void AD_RD_MultiReg(SPI_TypeDef* SPIx, uint8_t reg, uint8_t regnum, uint8_t *rBuf);
void AD_WR_MultiReg(SPI_TypeDef* SPIx, uint8_t reg, uint8_t regnum, uint8_t *wBuf);
void ADC_Parameter_Config(void);
void ADC_Parameter_Config_Selftest_start(void);
void Stop_RDC(void);
void Self_Cal(void);
void Self_OCal(void);
void Self_GCal(void);
void Sys_OCal(void);
void Sys_GCal(void);
void SYN_CON(void);
void Standby_Mode(void);
void ADS1255SetGain(uint8_t GainCode);
void ADS1255SetRate(uint8_t Rate);
void ADS1255SetMux(uint8_t Mux);
void ADS1255SetMux_sigle(SPI_TypeDef* SPIx,uint8_t Mux);
void ADC_Config_RFLASH(void);

//预采集数据结构
typedef struct
{
    int currentPointer;//当前指针（指向当前可写序号）
    //uint8_t preData_X[128*3];
    //uint8_t preData_Y[128*3];
    //uint8_t preData_Z[128*3];
    int preSamplingSwitch;
	  int syncSamplingPosition;
	  int syncTriggerStatus;
}PreSamplingData;
typedef struct
{
    uint8_t numOfPoint_level;//0x01
    uint8_t sampleFrequency_level;//0x03
    uint8_t preSamplingTime_points;//0x00
	  uint8_t delayTime_ms;//0x00
	  uint8_t headGain;//0x01
	  
}SETS;
extern PreSamplingData pre_sample;
extern int syncTriggerStatus;
extern SETS sets;

/*===========================ADC外部接口函数?=========================*/
//void AD_Contine_Convertion(uint8_t *Data_X, uint8_t *Data_Y, uint8_t *Data_Z, s32 Con_Num);
void AD_RD_Data_Continous(uint8_t *Data_X, uint8_t *Data_Y, uint8_t *Data_Z, s32 Con_Num);
void ad_modules_init(void);
void ADC_init(void);
void ADS1255_RESET(void);
//预采集相关函数
void pre_sample_init(void);
void start_pre_sample(void);
void pre_sample_process(void);
void stop_pre_sample(void);
void Pre_sample_control(FunctionalState state);
void sample_process_exti(void);
//AD测试函数
void AD_TEST(void);
void Filter_DC(uint8_t * data,int data_length);

void Delay_us(s32 time_us);
void ADC_Parameter_Set(void);
void ADC_Parameter_Save_WFLASH(void);
void ADC_Parameter_Config_RFLASH(void);

//*******************************************************************************************//
//***********************************************************************************************//

#endif

/*
  
* All rights reserved.
*
* File Name          : direction.c
* Description        : ��������ز���
*
* Version            : V1.0
* Author             : liu Yong
* Date               : 7/7/2012
*
* OriginalVersion    : V1.0
* Original Author    : liu Yong
* Date               : 7/7/2012
*/
#ifndef __DIRECTION_H__
#define __DIRECTION_H__

#include "stm32f4xx.h"
#include "board_init.h"

//λ������,ʵ��51���Ƶ�GPIO���ƹ���
//����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).M4ͬM3����,ֻ�ǼĴ�����ַ����.
//IO�ڲ����궨��
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 

//IO�ڵ�ַӳ��
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414  
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 
//************************************
extern unsigned char current_direction[12];


extern  uint8_t ACC_CALSW[3];//���ٶ�У׼
extern  uint8_t MAG_CALSW[3];//�ų�У׼
extern  uint8_t CALSW_QUIT[3];//�˳�У׼

extern  uint8_t AXOFF[3];//���ٶ�ƫ������
extern  uint8_t AYOFF[3];//
extern  uint8_t AZOFF[3];//

extern  uint8_t HXOFF[3];//�ų�ƫ������
extern  uint8_t HYOFF[3];//
extern  uint8_t HZOFF[3];//

extern  uint8_t SAVE_CONFIG[3];//���浱ǰ����
extern  uint8_t RESET_CONFIG[3];//�ָ�Ĭ������

extern  uint8_t ALG9[3];//axis9�㷨
extern  uint8_t ALG6[3];//axis6

extern  uint8_t DIRECTION_HOR[3];//ˮƽ��װ
extern  uint8_t DIRECTION_VER[3];//��ֱ��װ

extern  uint8_t BANDWIDTH[3];//20hz����
extern	uint8_t BANDWIDTH_256[3];//256hz����
extern  uint8_t BANDWIDTH_188[3];//188hz����
extern  uint8_t BANDWIDTH_98[3];//98hz����
extern  uint8_t BANDWIDTH_42[3];//42hz����
extern  uint8_t BANDWIDTH_10[3];//10hz����
extern  uint8_t BANDWIDTH_5[3];//5hz����
 
extern  uint8_t ACCRANGE[3];//���ٶ�2g/s2
extern  uint8_t ACCRANGE_4g[3];//���ٶ�4g/s2
extern  uint8_t ACCRANGE_8g[3];//���ٶ�8g/s2
extern  uint8_t ACCRANGE_16g[3];//���ٶ�16g/s2
 			
extern  uint8_t GYRORANGE[3];//������250deg/s
extern  uint8_t GYRORANGE_500[3];//������500deg/s
extern  uint8_t GYRORANGE_1000[3];//������1000deg/s
extern  uint8_t GYRORANGE_2000[3];//������2000deg/s

extern 	uint8_t direction_data[30];//��Ÿ�������

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 
//��ģ������ GPIOB10->SCL GPIOB11->SDA  
/*ģ��IIC�˿�������붨��*/
#define SCL_H         GPIO_SetBits(GPIOB , GPIO_Pin_10)
#define SCL_L         GPIO_ResetBits(GPIOB , GPIO_Pin_10)

#define SDA_H         GPIO_SetBits(GPIOB , GPIO_Pin_11)
#define SDA_L         GPIO_ResetBits(GPIOB , GPIO_Pin_11)


#define SCL_read      GPIOB->IDR  & GPIO_Pin_10
#define READ_SDA      GPIOB->IDR  & GPIO_Pin_11

//IO��������
#define SDA_IN()  {GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=0<<11*2;}	//PB11����ģʽ
#define SDA_OUT() {GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=1<<11*2;} //PB11���ģʽ



#define IIC_SCL    PBout(10) //SCL
#define IIC_SDA    PBout(11) //SDA


#define SAVE 			0x00
#define CALSW 		0x01
#define RSW 			0x02
#define RRATE			0x03
#define BAUD 			0x04
#define AXOFFSET	0x05
#define AYOFFSET	0x06
#define AZOFFSET	0x07
#define GXOFFSET	0x08
#define GYOFFSET	0x09
#define GZOFFSET	0x0a
#define HXOFFSET	0x0b
#define HYOFFSET	0x0c
#define HZOFFSET	0x0d
#define D0MODE		0x0e
#define D1MODE		0x0f
#define D2MODE		0x10
#define D3MODE		0x11
#define D0PWMH		0x12
#define D1PWMH		0x13
#define D2PWMH		0x14
#define D3PWMH		0x15
#define D0PWMT		0x16
#define D1PWMT		0x17
#define D2PWMT		0x18
#define D3PWMT		0x19
#define IICADDR		0x1a
#define LEDOFF 		0x1b
#define GPSBAUD		0x1c

#define YYMM				0x30
#define DDHH				0x31
#define MMSS				0x32
#define MS					0x33
#define AX					0x34
#define AY					0x35
#define AZ					0x36
#define GX					0x37
#define GY					0x38
#define GZ					0x39
#define HX					0x3a
#define HY					0x3b
#define HZ					0x3c			
#define Roll				0x3d
#define Pitch				0x3e
#define Yaw					0x3f
#define TEMP				0x40
#define D0Status		0x41
#define D1Status		0x42
#define D2Status		0x43
#define D3Status		0x44
#define PressureL		0x45
#define PressureH		0x46
#define HeightL			0x47
#define HeightH			0x48
#define LonL				0x49
#define LonH				0x4a
#define LatL				0x4b
#define LatH				0x4c
#define GPSHeight   0x4d
#define GPSYAW      0x4e
#define GPSVL				0x4f
#define GPSVH				0x50
      
#define DIO_MODE_AIN 0
#define DIO_MODE_DIN 1
#define DIO_MODE_DOH 2
#define DIO_MODE_DOL 3
#define DIO_MODE_DOPWM 4
#define DIO_MODE_GPS 5			

//****************************
/*I2C��������*/
void I2C_GPIO_Config(void);
void I2C_delay(u32 count);
void Delayms(vu32 m);
void DELAY_MS(int ms);
void I2C_Start(void);
void I2C_Stop(void);
u8 I2C_WaitAck(void); 	 //����Ϊ:=1��ACK,=0��ACK
void I2C_SendByte(u8 SendByte); //���ݴӸ�λ����λ//

u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data);
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data);
//��Ҫ�ⲿ���ýӿ�
void init_direction(void);
void get_direction(void);
//void direction_test(void);
void Direction_init(void);

void direction_test(void);
short CharToShort(unsigned char cData[]);
u8 IIC_Read_Byte(unsigned char ack);	



#endif /* __DIRECTION_H__ */


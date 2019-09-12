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

/* Scheduler includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "direction.h"
#include "string.h"
#define DIRECTION_THREAD_PRIO    ( tskIDLE_PRIORITY + 5 )

//ȫ�ֱ�������
unsigned char current_direction[12];
float Angle[3];
unsigned char str[100];

 uint8_t ACC_CALSW[3]={0x01,0x01,0x00};//���ٶ�У׼
 uint8_t MAG_CALSW[3]={0x01,0x02,0x00};//�ų�У׼
 uint8_t CALSW_QUIT[3]={0x01,0x00,0x00};//�˳�У׼

 uint8_t AXOFF[3]={0x05,0x00,0x00};//���ٶ�ƫ������
 uint8_t AYOFF[3]={0x06,0x00,0x00};//
 uint8_t AZOFF[3]={0x07,0x00,0x00};//

 uint8_t HXOFF[3]={0x0b,0x00,0x00};//�ų�ƫ������
 uint8_t HYOFF[3]={0x0c,0x00,0x00};//
 uint8_t HZOFF[3]={0x0d,0x00,0x00};//

 uint8_t SAVE_CONFIG[3]={0x00,0x00,0x00};//���浱ǰ����
 uint8_t RESET_CONFIG[3]={0x00,0x01,0x00};//�ָ�Ĭ������

 uint8_t ALG9[3]={0x24,0x00,0x00};//axis9�㷨
 uint8_t ALG6[3]={0x24,0x01,0x00};//axis6�㷨
 
 uint8_t DIRECTION_HOR[3]={0x23,0x00,0x00};//ˮƽ��װ
 uint8_t DIRECTION_VER[3]={0x23,0x01,0x00};//��ֱ��װ
 
 uint8_t BANDWIDTH[3]={0x1f,0x04,0x00};//20hz����
 uint8_t BANDWIDTH_256[3]={0x1f,0x00,0x00};//256hz����
 uint8_t BANDWIDTH_188[3]={0x1f,0x01,0x00};//188hz����
 uint8_t BANDWIDTH_98[3]={0x1f,0x02,0x00};//98hz����
 uint8_t BANDWIDTH_42[3]={0x1f,0x03,0x00};//42hz����
 uint8_t BANDWIDTH_10[3]={0x1f,0x05,0x00};//10hz����
 uint8_t BANDWIDTH_5[3]={0x1f,0x06,0x00};//5hz����
 
 
 uint8_t ACCRANGE[3]={0x21,0x00,0x00};//���ٶ�2g/s2
 uint8_t ACCRANGE_4g[3]={0x21,0x01,0x00};//���ٶ�4g/s2
 uint8_t ACCRANGE_8g[3]={0x21,0x02,0x00};//���ٶ�8g/s2
 uint8_t ACCRANGE_16g[3]={0x21,0x03,0x00};//���ٶ�16g/s2
 			
 
 uint8_t GYRORANGE[3]={0x20,0x00,0x00};//������250deg/s
 uint8_t GYRORANGE_500[3]={0x20,0x01,0x00};//������500deg/s
 uint8_t GYRORANGE_1000[3]={0x20,0x02,0x00};//������1000deg/s
 uint8_t GYRORANGE_2000[3]={0x20,0x03,0x00};//������2000deg/s

 uint8_t direction_data[30];//��Ÿ�������
	

/*******************************************************************************
* Function Name  : I2C_GPIO_Config
* Description    : Configration Simulation IIC GPIO
* Input          : None 
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_GPIO_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��
  //GPIOB8,B9��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
	SDA_OUT();     //sda�����	
	
	SDA_H;
	SCL_H;
}


/*******************************************************************************
* Function Name  : I2C_delay
* Description    : Simulation IIC Timing series delay
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_delay(u32 count)
{
	  u32 count_Temp= count*10; 
		while (count_Temp--);
}


/*******************************************************************************
* Function Name  : I2C_Start
* Description    : None
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void I2C_Start(void)
{
		SDA_OUT();     //sda�����
    SDA_H;
    SCL_H;
    I2C_delay(5);
    SDA_L;
    I2C_delay(5);
    SCL_L;//ǯסI2C���ߣ�׼�����ͻ��������    
}
/*******************************************************************************
* Function Name  : I2C_Stop
* Description    : Master Stop Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Stop(void)
{
  SDA_OUT();//sda�����
	SCL_L;
	SDA_L;//STOP:when CLK is high DATA change form low to high
 	I2C_delay(5);
	SCL_H;
	SDA_H; //����I2C���߽����ź�
	I2C_delay(5);						   	
} 

 
/*******************************************************************************
* Function Name  : I2C_WaitAck
* Description    : None
* Input          : None
* Output         : None
* Return         : ����Ϊ:=1��ACK,=0��ACK
* Attention		 : None
*******************************************************************************/
u8 I2C_WaitAck(void) 	 //����Ϊ:=1��ACK,=0��ACK
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	SDA_H;	   
	I2C_delay(5);
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			I2C_Stop();
			return 1;
		}
		I2C_delay(5);
	}
	SCL_H;//ʱ�����1 
	I2C_delay(5);
	SCL_L;//ʱ�����0  
	return 0;  
}
/*******************************************************************************
* Function Name  : I2C_SendByte
* Description    : Master Send a Byte to Slave
* Input          : Will Send Date
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_SendByte(u8 SendByte) //���ݴӸ�λ����λ//
{
    u8 t;   
	  SDA_OUT(); 	    
    SCL_L;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(SendByte&0x80)>>7;
        SendByte<<=1; 	  
				I2C_delay(2);   //��TEA5767��������ʱ���Ǳ����
				SCL_H;
				I2C_delay(5);
				SCL_L;	
				I2C_delay(3);
    }	 
} 

/********************************************************************************
** �������� �� void Delayms(vu32 m)
** �������� �� ����ʱ����	 m=1,��ʱ1ms
** ��    ��	�� ��
** ��    ��	�� ��
** ��    ��	�� ��
********************************************************************************
*/
void Delayms(vu32 m)
{
    u32 i;
    
    for(; m != 0; m--)	
        //for (i=0; i<50000; i++);
    for (i=0; i<150000; i++);
}

void DELAY_MS(int ms)
{
	int i,j,k;
	for(k=0;k<ms;k++){
		for(i=0;i<168;i++)
		{
			for(j=0;j<250;j++);
		}
	}

}



short CharToShort(unsigned char cData[])
{
	return ((short)cData[1]<<8)|cData[0];
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*��������:	��ȡָ���豸 ָ���Ĵ����� length��ֵ
*����	dev  Ŀ���豸��ַ
		  reg	  �Ĵ�����ַ
		  length Ҫ�����ֽ���
		  *data  ���������ݽ�Ҫ��ŵ�ָ��
����  ���������ֽ�����
*******************************************************************************/ 
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
{
    u8 count = 0;	
	  I2C_Start();
  	I2C_SendByte(dev<<1);	   //����д����
	  I2C_WaitAck();
	  I2C_SendByte(reg);   //���͵�ַ
    I2C_WaitAck();	  
	  I2C_Start();
	  I2C_SendByte((dev<<1)+1);  //�������ģʽ	
	  I2C_WaitAck();	
	
		for(count=0;count<length;count++)
		{
				if(count!=length-1)
				{
					data[count]=IIC_Read_Byte(1);  //��ACK�Ķ�����
				}
				else
				{
					data[count]=IIC_Read_Byte(0);	 //���һ���ֽ�NACK
				}
		}
    I2C_Stop();//����һ��ֹͣ����
    return count;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Ack(void)
*��������:	    ����ACKӦ��
*******************************************************************************/
void IIC_Ack(void)
{
		SCL_L;
		SDA_OUT();
		SDA_L;
		I2C_delay(5);
		SCL_H;
		I2C_delay(5);
		SCL_L;
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_NAck(void)
*��������:	    ����NACKӦ��
*******************************************************************************/	    
void IIC_NAck(void)
{
		SCL_L;
		SDA_OUT();
		IIC_SDA=1;
		I2C_delay(5);
		SCL_H;
		I2C_delay(5);
		SCL_L;
}			
/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IIC_Read_Byte(unsigned char ack)
*��������:	    //��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK 
*******************************************************************************/  
u8 IIC_Read_Byte(unsigned char ack)
{		
		unsigned char i,receive=0;
		SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
		{
        SCL_L; 
        I2C_delay(5);	
	    	SCL_H;
        receive<<=1;
        if(READ_SDA)receive++;   
		    I2C_delay(5);	
    }					 
    if (ack)
		{
        IIC_Ack();//����nACK
		}
    else
		{
        IIC_NAck(); //����ACK   
		}
    return receive;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
*��������:	    ������ֽ�д��ָ���豸 ָ���Ĵ���
����	dev  Ŀ���豸��ַ
		reg	  �Ĵ�����ַ
		length Ҫд���ֽ���
		*data  ��Ҫд�����ݵ��׵�ַ
����   �����Ƿ�ɹ�
*******************************************************************************/ 
u8 IICwriteBytes(u8 dev, u8 reg, u8 length,  u8* data)
{
  
 	u8 count = 0;
	I2C_Start();
	I2C_SendByte(dev<<1);	   //����д����
	I2C_WaitAck();
	I2C_SendByte(reg);   //���͵�ַ
	I2C_WaitAck();	  
	for(count=0;count<length;count++){
		I2C_SendByte(data[count]); 
		I2C_WaitAck(); 
 }
	I2C_Stop();//����һ��ֹͣ����

    return 1; //status == 0;
	
}
/********************************************************************************
** �������� �� init_direction
** �������� �� ��ʼ�������ǵ�
** ��    ��	�� ��
** ��    ��	�� ��
** ��    ��	�� ��
********************************************************************************
*/
void init_direction()
{
    I2C_GPIO_Config();
    Delayms(10); 
		memset(direction_data,0,30);
}
/********************************************************************************
** �������� �� direction_test
** �������� �� ��ȡ��ǰʱ��������״̬��Ϣ�������DIRECTION_DATA�У�ǰ����Ϊ���ٶȴ�������������Ϊ������
** ��    ��	�� ��
** ��    ��	�� ��
** ��    ��	�� ��
********************************************************************************
*/
void direction_test(void)
{   
// 	  get_direction();
//		��ϵͳ�о������ٸ������㣬����λ���н���
//	  Angle[0] = (float)CharToShort(&current_direction[0])/32768*180;
//		Angle[1] = (float)CharToShort(&current_direction[2])/32768*180;
//		Angle[2] = (float)CharToShort(&current_direction[4])/32768*180;	
//	  sprintf((char*)str,"0x50: Angle:%.3f  %.3f  %.3f \r\n",Angle[0],Angle[1],Angle[2]);
//	  printf(str);
}
/********************************************************************************
** �������� �� get_direction
** �������� �� ��ȡ��λ��Ϣ
** ��    ��	�� ��
** ��    ��	�� ��
** ��    ��	�� ��
********************************************************************************
*/
void get_direction(void)
{
	
  IICreadBytes(0x50, Roll, 6,&current_direction[0]);
}

/**
  * @brief  Direction thread 
  * @param arg: pointer on argument(not used here) 
  * @retval None
  */
static void Direction_thread(void *arg)
{
  while (1) 
  {
		get_direction();
		vTaskDelay(1000);
  }
}
//    ��������Direction_init
//    ��  ��: 
//    ��  ��:
//    ����˵����
//*/
void Direction_init()
{
	 xTaskCreate(Direction_thread, (int8_t *) "Direct", configMINIMAL_STACK_SIZE * 5, NULL,DIRECTION_THREAD_PRIO, NULL);
}

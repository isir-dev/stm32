/*
  
* All rights reserved.
*
* File Name          : direction.c
* Description        : 陀螺仪相关操作
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

//全局变量声明
unsigned char current_direction[12];
float Angle[3];
unsigned char str[100];

 uint8_t ACC_CALSW[3]={0x01,0x01,0x00};//加速度校准
 uint8_t MAG_CALSW[3]={0x01,0x02,0x00};//磁场校准
 uint8_t CALSW_QUIT[3]={0x01,0x00,0x00};//退出校准

 uint8_t AXOFF[3]={0x05,0x00,0x00};//加速度偏移置零
 uint8_t AYOFF[3]={0x06,0x00,0x00};//
 uint8_t AZOFF[3]={0x07,0x00,0x00};//

 uint8_t HXOFF[3]={0x0b,0x00,0x00};//磁场偏移置零
 uint8_t HYOFF[3]={0x0c,0x00,0x00};//
 uint8_t HZOFF[3]={0x0d,0x00,0x00};//

 uint8_t SAVE_CONFIG[3]={0x00,0x00,0x00};//保存当前配置
 uint8_t RESET_CONFIG[3]={0x00,0x01,0x00};//恢复默认设置

 uint8_t ALG9[3]={0x24,0x00,0x00};//axis9算法
 uint8_t ALG6[3]={0x24,0x01,0x00};//axis6算法
 
 uint8_t DIRECTION_HOR[3]={0x23,0x00,0x00};//水平安装
 uint8_t DIRECTION_VER[3]={0x23,0x01,0x00};//垂直安装
 
 uint8_t BANDWIDTH[3]={0x1f,0x04,0x00};//20hz带宽
 uint8_t BANDWIDTH_256[3]={0x1f,0x00,0x00};//256hz带宽
 uint8_t BANDWIDTH_188[3]={0x1f,0x01,0x00};//188hz带宽
 uint8_t BANDWIDTH_98[3]={0x1f,0x02,0x00};//98hz带宽
 uint8_t BANDWIDTH_42[3]={0x1f,0x03,0x00};//42hz带宽
 uint8_t BANDWIDTH_10[3]={0x1f,0x05,0x00};//10hz带宽
 uint8_t BANDWIDTH_5[3]={0x1f,0x06,0x00};//5hz带宽
 
 
 uint8_t ACCRANGE[3]={0x21,0x00,0x00};//加速度2g/s2
 uint8_t ACCRANGE_4g[3]={0x21,0x01,0x00};//加速度4g/s2
 uint8_t ACCRANGE_8g[3]={0x21,0x02,0x00};//加速度8g/s2
 uint8_t ACCRANGE_16g[3]={0x21,0x03,0x00};//加速度16g/s2
 			
 
 uint8_t GYRORANGE[3]={0x20,0x00,0x00};//陀螺仪250deg/s
 uint8_t GYRORANGE_500[3]={0x20,0x01,0x00};//陀螺仪500deg/s
 uint8_t GYRORANGE_1000[3]={0x20,0x02,0x00};//陀螺仪1000deg/s
 uint8_t GYRORANGE_2000[3]={0x20,0x03,0x00};//陀螺仪2000deg/s

 uint8_t direction_data[30];//存放更新数据
	

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
  //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟
  //GPIOB8,B9初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	SDA_OUT();     //sda线输出	
	
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
		SDA_OUT();     //sda线输出
    SDA_H;
    SCL_H;
    I2C_delay(5);
    SDA_L;
    I2C_delay(5);
    SCL_L;//钳住I2C总线，准备发送或接收数据    
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
  SDA_OUT();//sda线输出
	SCL_L;
	SDA_L;//STOP:when CLK is high DATA change form low to high
 	I2C_delay(5);
	SCL_H;
	SDA_H; //发送I2C总线结束信号
	I2C_delay(5);						   	
} 

 
/*******************************************************************************
* Function Name  : I2C_WaitAck
* Description    : None
* Input          : None
* Output         : None
* Return         : 返回为:=1有ACK,=0无ACK
* Attention		 : None
*******************************************************************************/
u8 I2C_WaitAck(void) 	 //返回为:=1有ACK,=0无ACK
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
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
	SCL_H;//时钟输出1 
	I2C_delay(5);
	SCL_L;//时钟输出0  
	return 0;  
}
/*******************************************************************************
* Function Name  : I2C_SendByte
* Description    : Master Send a Byte to Slave
* Input          : Will Send Date
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_SendByte(u8 SendByte) //数据从高位到低位//
{
    u8 t;   
	  SDA_OUT(); 	    
    SCL_L;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(SendByte&0x80)>>7;
        SendByte<<=1; 	  
				I2C_delay(2);   //对TEA5767这三个延时都是必须的
				SCL_H;
				I2C_delay(5);
				SCL_L;	
				I2C_delay(3);
    }	 
} 

/********************************************************************************
** 函数名称 ： void Delayms(vu32 m)
** 函数功能 ： 长延时函数	 m=1,延时1ms
** 输    入	： 无
** 输    出	： 无
** 返    回	： 无
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

/**************************实现函数********************************************
*函数原型:	u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*功　　能:	读取指定设备 指定寄存器的 length个值
*输入	dev  目标设备地址
		  reg	  寄存器地址
		  length 要读的字节数
		  *data  读出的数据将要存放的指针
返回  读出来的字节数量
*******************************************************************************/ 
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
{
    u8 count = 0;	
	  I2C_Start();
  	I2C_SendByte(dev<<1);	   //发送写命令
	  I2C_WaitAck();
	  I2C_SendByte(reg);   //发送地址
    I2C_WaitAck();	  
	  I2C_Start();
	  I2C_SendByte((dev<<1)+1);  //进入接收模式	
	  I2C_WaitAck();	
	
		for(count=0;count<length;count++)
		{
				if(count!=length-1)
				{
					data[count]=IIC_Read_Byte(1);  //带ACK的读数据
				}
				else
				{
					data[count]=IIC_Read_Byte(0);	 //最后一个字节NACK
				}
		}
    I2C_Stop();//产生一个停止条件
    return count;
}

/**************************实现函数********************************************
*函数原型:		void IIC_Ack(void)
*功　　能:	    产生ACK应答
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
/**************************实现函数********************************************
*函数原型:		void IIC_NAck(void)
*功　　能:	    产生NACK应答
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
/**************************实现函数********************************************
*函数原型:		u8 IIC_Read_Byte(unsigned char ack)
*功　　能:	    //读1个字节，ack=1时，发送ACK，ack=0，发送nACK 
*******************************************************************************/  
u8 IIC_Read_Byte(unsigned char ack)
{		
		unsigned char i,receive=0;
		SDA_IN();//SDA设置为输入
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
        IIC_Ack();//发送nACK
		}
    else
		{
        IIC_NAck(); //发送ACK   
		}
    return receive;
}

/**************************实现函数********************************************
*函数原型:		u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
*功　　能:	    将多个字节写入指定设备 指定寄存器
输入	dev  目标设备地址
		reg	  寄存器地址
		length 要写的字节数
		*data  将要写的数据的首地址
返回   返回是否成功
*******************************************************************************/ 
u8 IICwriteBytes(u8 dev, u8 reg, u8 length,  u8* data)
{
  
 	u8 count = 0;
	I2C_Start();
	I2C_SendByte(dev<<1);	   //发送写命令
	I2C_WaitAck();
	I2C_SendByte(reg);   //发送地址
	I2C_WaitAck();	  
	for(count=0;count<length;count++){
		I2C_SendByte(data[count]); 
		I2C_WaitAck(); 
 }
	I2C_Stop();//产生一个停止条件

    return 1; //status == 0;
	
}
/********************************************************************************
** 函数名称 ： init_direction
** 函数功能 ： 初始化陀螺仪等
** 输    入	： 无
** 输    出	： 无
** 返    回	： 无
********************************************************************************
*/
void init_direction()
{
    I2C_GPIO_Config();
    Delayms(10); 
		memset(direction_data,0,30);
}
/********************************************************************************
** 函数名称 ： direction_test
** 函数功能 ： 读取当前时刻陀螺仪状态信息，存放于DIRECTION_DATA中，前三组为加速度传感器，后四组为陀螺仪
** 输    入	： 无
** 输    出	： 无
** 返    回	： 无
********************************************************************************
*/
void direction_test(void)
{   
// 	  get_direction();
//		在系统中尽量减少浮点运算，在上位机中进行
//	  Angle[0] = (float)CharToShort(&current_direction[0])/32768*180;
//		Angle[1] = (float)CharToShort(&current_direction[2])/32768*180;
//		Angle[2] = (float)CharToShort(&current_direction[4])/32768*180;	
//	  sprintf((char*)str,"0x50: Angle:%.3f  %.3f  %.3f \r\n",Angle[0],Angle[1],Angle[2]);
//	  printf(str);
}
/********************************************************************************
** 函数名称 ： get_direction
** 函数功能 ： 获取方位信息
** 输    入	： 无
** 输    出	： 无
** 返    回	： 无
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
//    函数名：Direction_init
//    输  入: 
//    输  出:
//    功能说明：
//*/
void Direction_init()
{
	 xTaskCreate(Direction_thread, (int8_t *) "Direct", configMINIMAL_STACK_SIZE * 5, NULL,DIRECTION_THREAD_PRIO, NULL);
}

/*
  
* All rights reserved.
*
* File Name          : rs485.c
* Description        :  PPP串行网络协议实现
*
* Version            : V1.0
* Author             : liu Yong
* Date               : 7/19/2012
*
* OriginalVersion    : V1.0
* Original Author    : liu Yong
* Date               : 7/19/2012
*/

/* Scheduler includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "rs485.h"
#include "CRC_8.h"
#include "main.h"
//#include "led.h"
#include "direction.h"
#include "uart.h"
#include "ads1255.h"
///*******************************************************************************
//    全局结构声明
//*/
struct PPP_FRAME_STRUCT ppp_frame_struct;
uint8_t PPP_send_frame[MAX_FRAME];//PPP发送缓存

///*******************************************************************************
//    本文件内部相关函数声明
//*/
void ppp_rs485_configuration_host(void)
{ 
    NVIC_InitTypeDef NVIC_InitStructure;
    USART_InitTypeDef USART_InitStructure; 
    
    
    GPIO_InitTypeDef GPIO_InitType;	    /* GPIO端口结构体变量 */
    /*-------------------------------Alternate function-----------*/
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
    
    /********************USART->TX, USART->RX*********************/
    GPIO_InitType.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; 					    
    GPIO_InitType.GPIO_Mode = GPIO_Mode_AF;	  		/* 复用功能 */
    GPIO_InitType.GPIO_Speed = GPIO_Speed_50MHz;	    /* 口线翻转速度为50MHz */
    GPIO_InitType.GPIO_PuPd = GPIO_PuPd_UP;		    /* PULL up */
    GPIO_InitType.GPIO_OType = GPIO_OType_PP;	        /* Push-pull mode */
    GPIO_Init(GPIOB, &GPIO_InitType);		          	
    
    //USART_InitStructure.USART_BaudRate = 57600;//实际为115200
//    USART_InitStructure.USART_BaudRate = 115200;//实际为230400
//    USART_InitStructure.USART_BaudRate = 288000;//实际为576000
		USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    USART_Init(USART1, &USART_InitStructure); 
    USART_ClearFlag(USART1,USART_FLAG_TC);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    //USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    USART_Cmd(USART1, ENABLE);
    
    /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    
    /* Enable the USART1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			     	//设置串口中断
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    ppp_frame_struct.index_frame = 0;//PPP帧内部指针
    ppp_frame_struct.escape_character[0]=0;//PPP帧转义字符分析器
    ppp_frame_struct.escape_character[1]=0;//PPP帧转义字符分析器
    ppp_frame_struct.flags_frame=0;//PPP接收中断
    ppp_frame_struct.length_frame=0;//PPP帧长度
		ppp_frame_struct.currt_frame=currt_frame1;
		
    //rs485收发控制引脚初始化
    GPIO_InitType.GPIO_Pin = GPIO_Pin_13;	         
    GPIO_InitType.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitType.GPIO_OType = GPIO_OType_PP;	  		/* 推挽输出	*/
    GPIO_InitType.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_Init(GPIOC, &GPIO_InitType);	
    
    RS485_SET_RX_MODE;
}

///*******************************************************************************
//    函数名：ppp_rs485_configuration
//    输  入: 
//    输  出:
//    功能说明：基于串口的PPP通信协议底层硬件配置
//*/
void ppp_rs485_configuration(void)
{ 
    ppp_rs485_configuration_host();
    RS485_SET_RX_MODE;
}

/*******************************************************************************
    函数名：CRC_Calculate_PPP
    输  入: 
    输  出:
    功能说明：CRC校验码计算
*/
uint8_t CRC_Calculate_PPP(uint8_t *vector, uint16_t Length)	 /* Length为信息位长度 */
{
    // 	uint16_t i;
    // 	uint8_t val=0;
    // 	for(i=0;i<Length;i++)
    // 	{
    // 		val = val^vector[i];
    // 	}
    //return vector[Length-1];			
    //return val;
    return CRC8_Tab(vector,Length);	
}
/*******************************************************************************
    函数名：check_ppp_frame
    输  入: 
    输  出:
    功能说明：校验接收PPP帧是否完整
*/
uint8_t check_ppp_frame(void)
{
	uint8_t value_ret=0;
	switch(ppp_frame_struct.currt_frame) 
	{			
		case currt_frame1:
			if(ppp_frame_struct.frame1[ppp_frame_struct.index_frame-1] == 
            CRC_Calculate_PPP(PPP_recv_frame1,ppp_frame_struct.index_frame-2))
			{
        value_ret = 1;
			}
			else
      {
        value_ret = 0;
      }
			break;
		case currt_frame2:
			if(ppp_frame_struct.frame2[ppp_frame_struct.index_frame-1] == 
            CRC_Calculate_PPP(PPP_recv_frame2,ppp_frame_struct.index_frame-2))
			{
        value_ret = 1;
			}
			else
      {
        value_ret = 0;
      }
			break;
		case currt_frame3:
			if(ppp_frame_struct.frame3[ppp_frame_struct.index_frame-1] == 
            CRC_Calculate_PPP(PPP_recv_frame3,ppp_frame_struct.index_frame-2))
			{
        value_ret = 1;
			}
			else
      {
        value_ret = 0;
      }
			break;
		default:
			value_ret = 0;
			break;
	}
	return value_ret;
}

/*******************************************************************************
    函数名：char_process
    输  入: 
    输  出:
    功能说明：基于串口2的PPP通信协议底层硬件外设模拟
*/
void ppp_char_process(void)
{
    uint8_t recev_char;
	  uint8_t frame_type;
	  Uart_Queue_Struct temp_Uart_Queue;
    recev_char = USART_ReceiveData(RS485_USART);   //将读寄存器的数据缓存到接收缓冲区里
    if(recev_char == 0x7E)//分组的开始或结束
    {
        if(ppp_frame_struct.index_frame && (check_ppp_frame() == 1))//收到完整的一帧数据
        {
//					  SDA_H;
//					  Delayms(1);
//					  SDA_L;
					
            if(ppp_frame_struct.index_frame>=MAX_FRAME)
						{
								ppp_frame_struct.index_frame=0;
								return ;
						}
            ppp_frame_struct.length_frame = ppp_frame_struct.index_frame-2;//PPP帧长度不包括头、尾间隔符、校验字节
            ppp_frame_struct.index_frame=0;//指向下一个要存储的位置
						
						switch(ppp_frame_struct.currt_frame) 
						{
							case currt_frame1:
								if((ppp_frame_struct.flags_frame & 0x02) != 0x02)
								{
									ppp_frame_struct.frame1[ppp_frame_struct.index_frame]=recev_char;	
									ppp_frame_struct.currt_frame=currt_frame2;
									ppp_frame_struct.flags_frame = ppp_frame_struct.flags_frame|0x01;//标志帧收到，需要上层协议来读取处理
									temp_Uart_Queue.length=ppp_frame_struct.length_frame;
									temp_Uart_Queue.data=PPP_recv_frame1;
									temp_Uart_Queue.currt_frame=currt_frame1;		
									//xQueueSendToBackFromISR(xQueue_Uart, &temp_Uart_Queue, 0 );	
									frame_type=temp_Uart_Queue.data[2];	
									if(frame_type==SYNC_REQUEST)
									{
										ppp_frame_struct.flags_frame = ppp_frame_struct.flags_frame &0xFE;
										pre_sample.syncSamplingPosition=pre_sample.currentPointer;
										pre_sample.syncTriggerStatus=1;
										//SDA_L;
									}	
									else									
									{
										xQueueSendToBackFromISR(xQueue_Uart, &temp_Uart_Queue, 0 );		
									}							
								}
								break;
							case currt_frame2:
								if((ppp_frame_struct.flags_frame & 0x04) != 0x04)
								{
									ppp_frame_struct.frame2[ppp_frame_struct.index_frame]=recev_char;	
									ppp_frame_struct.currt_frame=currt_frame3;
									ppp_frame_struct.flags_frame = ppp_frame_struct.flags_frame|0x02;//标志帧收到，需要上层协议来读取处理
									temp_Uart_Queue.length=ppp_frame_struct.length_frame;
									temp_Uart_Queue.data=PPP_recv_frame2;
									temp_Uart_Queue.currt_frame=currt_frame2;
									//xQueueSendToBackFromISR(xQueue_Uart, &temp_Uart_Queue, 0 );
                  frame_type=temp_Uart_Queue.data[2];	
									if(frame_type==SYNC_REQUEST)
									{
										ppp_frame_struct.flags_frame = ppp_frame_struct.flags_frame &0xFD;
										pre_sample.syncSamplingPosition=pre_sample.currentPointer;
										pre_sample.syncTriggerStatus=1;
										//SDA_L;
									}	
									else									
									{
										xQueueSendToBackFromISR(xQueue_Uart, &temp_Uart_Queue, 0 );		
									}												
								}
								break;
							case currt_frame3:
								if((ppp_frame_struct.flags_frame & 0x01) != 0x01)
								{
									ppp_frame_struct.frame3[ppp_frame_struct.index_frame]=recev_char;	
									ppp_frame_struct.currt_frame=currt_frame1;
									ppp_frame_struct.flags_frame = ppp_frame_struct.flags_frame|0x04;//标志帧收到，需要上层协议来读取处理
									temp_Uart_Queue.length=ppp_frame_struct.length_frame;
									temp_Uart_Queue.data=PPP_recv_frame3;
									temp_Uart_Queue.currt_frame=currt_frame3;
									//xQueueSendToBackFromISR(xQueue_Uart, &temp_Uart_Queue, 0 );	
                  frame_type=temp_Uart_Queue.data[2];	
									if(frame_type==SYNC_REQUEST)
									{
										ppp_frame_struct.flags_frame = ppp_frame_struct.flags_frame &0xFB;
										pre_sample.syncSamplingPosition=pre_sample.currentPointer;
										pre_sample.syncTriggerStatus=1;
										//SDA_L;
									}	
									else									
									{
										xQueueSendToBackFromISR(xQueue_Uart, &temp_Uart_Queue, 0 );		
									}												
								}
								break;
							default:
								break;
						}				
        }
        else if(ppp_frame_struct.index_frame && check_ppp_frame() == 0)//帧校验错,按新帧开头处理
        {
					  switch(ppp_frame_struct.currt_frame) 
						{
							case currt_frame1:
								ppp_frame_struct.frame1[0]=recev_char;
							  break;
							case currt_frame2:
									ppp_frame_struct.frame2[0]=recev_char;
							  break;
							case currt_frame3:
									ppp_frame_struct.frame3[0]=recev_char;
							  break;
							default:
							  break;
						}
            ppp_frame_struct.index_frame=1;//指向下一个要存储的位置		
        }
        else if(ppp_frame_struct.index_frame == 0)//收到帧头标志
        {
                switch(ppp_frame_struct.currt_frame) 
								{
									case currt_frame1:
										ppp_frame_struct.frame1[0]=recev_char;
										break;
									case currt_frame2:
										ppp_frame_struct.frame2[0]=recev_char;
										break;
									case currt_frame3:
										ppp_frame_struct.frame3[0]=recev_char;
										break;
									default:
									  break;
								}
                ppp_frame_struct.index_frame=1;//指向下一个要存储的位置		
        }
    }
    else if(recev_char == 0x7D)//收到转义字符
    {
        if(ppp_frame_struct.index_frame>0)//标志下一字节为转义字符
        {
            ppp_frame_struct.escape_character[0]=recev_char;
        }
    }
    else//收到帧内数据
    {
        if(ppp_frame_struct.escape_character[0]==0x7D)//收到转义字符
        {
            if(recev_char==0x5E)
            {
                recev_char=0x7E;
            }
            else if(recev_char==0x5D)
            {
                recev_char=0x7D;
            }
            else//未定义转义字符
            {
            }
            ppp_frame_struct.escape_character[0]=0;
        }
        if(0<ppp_frame_struct.index_frame && ppp_frame_struct.index_frame<MAX_FRAME-1)
        {
						switch(ppp_frame_struct.currt_frame) 
						{
							case currt_frame1:
								ppp_frame_struct.frame1[ppp_frame_struct.index_frame]=recev_char;
							  break;
							case currt_frame2:
								ppp_frame_struct.frame2[ppp_frame_struct.index_frame]=recev_char;
							  break;
							case currt_frame3:
								ppp_frame_struct.frame3[ppp_frame_struct.index_frame]=recev_char;
							  break;
							default:
							  break;
						}
            ppp_frame_struct.index_frame++;//指向下一个要存储的位
        }
    }
}

/*******************************************************************************
    函数名：ppp_send_frame
    输  入: 
    输  出:
    功能说明：基于串口2的PPP通信协议底层硬件发送帧函数
		2018-01-10 修改为先进性校验，然后发送数据
*/
uint16_t ppp_send_frame(uint8_t *source,uint16_t send_length)
{
    uint16_t i;
    uint8_t ppp_crc;
		//发送PPP帧校验
    ppp_crc = CRC_Calculate_PPP(source,send_length);
    //添加打开发送模式
    RS485_SET_TX_MODE;
    while(USART_GetFlagStatus(RS485_USART, USART_FLAG_TC)==RESET);
    USART_SendData(RS485_USART, 0x7E);	//发送PPP帧头
    for(i=0;i<send_length;i++)
    {
        if(source[i] == 0x7E)
        {
            while(USART_GetFlagStatus(RS485_USART, USART_FLAG_TC)==RESET);
            USART_SendData(RS485_USART, 0x7D);	
            while(USART_GetFlagStatus(RS485_USART, USART_FLAG_TC)==RESET);
            USART_SendData(RS485_USART, 0x5E);					
        }
        else if(source[i] == 0x7D)
        {
            while(USART_GetFlagStatus(RS485_USART, USART_FLAG_TC)==RESET);
            USART_SendData(RS485_USART, 0x7D);	
            while(USART_GetFlagStatus(RS485_USART, USART_FLAG_TC)==RESET);
            USART_SendData(RS485_USART, 0x5D);				
        }
        else
        {
            while(USART_GetFlagStatus(RS485_USART, USART_FLAG_TC)==RESET);
            USART_SendData(RS485_USART, source[i]);		
        }
    }
//    //发送PPP帧校验
//    ppp_crc = CRC_Calculate_PPP(source,send_length);
    if(ppp_crc == 0x7E)	
    {
        while(USART_GetFlagStatus(RS485_USART, USART_FLAG_TC)==RESET);
        USART_SendData(RS485_USART, 0x7D);	
        while(USART_GetFlagStatus(RS485_USART, USART_FLAG_TC)==RESET);
        USART_SendData(RS485_USART, 0x5E);					
    }
    else if(ppp_crc == 0x7D)
    {
        while(USART_GetFlagStatus(RS485_USART, USART_FLAG_TC)==RESET);
        USART_SendData(RS485_USART, 0x7D);	
        while(USART_GetFlagStatus(RS485_USART, USART_FLAG_TC)==RESET);
        USART_SendData(RS485_USART, 0x5D);				
    }
    else
    {
        while(USART_GetFlagStatus(RS485_USART, USART_FLAG_TC)==RESET);
        USART_SendData(RS485_USART, ppp_crc);	
    }
    //发送PPP帧尾
    while(USART_GetFlagStatus(RS485_USART, USART_FLAG_TC)==RESET);
    USART_SendData(RS485_USART, 0x7E);	//发送PPP帧尾
    while(USART_GetFlagStatus(RS485_USART, USART_FLAG_TC)==RESET);
    //添加关闭发送模式
    RS485_SET_RX_MODE;
    return send_length;
}

/*******************************************************************************
    函数名：ppp_send_data
    输  入: 
    输  出:
    功能说明:
*/
uint8_t ppp_send_data(uint16_t send_length)
{
	ppp_send_frame(PPP_send_frame,send_length);
	return 0;
}
/*******************************************************************************
    函数名：ppp_clear_isr_recv
    输  入: 
    输  出:
    功能说明:
*/
void ppp_clear_isr_recv(Uart_Queue_Struct uart_frame)
{
	switch(uart_frame.currt_frame) 
	{
		case currt_frame1:
			ppp_frame_struct.flags_frame = ppp_frame_struct.flags_frame &0xFE;
			break;
		case currt_frame2:
			ppp_frame_struct.flags_frame = ppp_frame_struct.flags_frame &0xFD;
			break;
		case currt_frame3:
			ppp_frame_struct.flags_frame = ppp_frame_struct.flags_frame &0xFB;
			break;
		default:
			break;
	}
}


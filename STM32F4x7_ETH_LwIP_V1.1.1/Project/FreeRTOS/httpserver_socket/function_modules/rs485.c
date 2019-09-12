/*
  
* All rights reserved.
*
* File Name          : rs485.c
* Description        :  PPP��������Э��ʵ��
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
//    ȫ�ֽṹ����
//*/
struct PPP_FRAME_STRUCT ppp_frame_struct;
uint8_t PPP_send_frame[MAX_FRAME];//PPP���ͻ���

///*******************************************************************************
//    ���ļ��ڲ���غ�������
//*/
void ppp_rs485_configuration_host(void)
{ 
    NVIC_InitTypeDef NVIC_InitStructure;
    USART_InitTypeDef USART_InitStructure; 
    
    
    GPIO_InitTypeDef GPIO_InitType;	    /* GPIO�˿ڽṹ����� */
    /*-------------------------------Alternate function-----------*/
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
    
    /********************USART->TX, USART->RX*********************/
    GPIO_InitType.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; 					    
    GPIO_InitType.GPIO_Mode = GPIO_Mode_AF;	  		/* ���ù��� */
    GPIO_InitType.GPIO_Speed = GPIO_Speed_50MHz;	    /* ���߷�ת�ٶ�Ϊ50MHz */
    GPIO_InitType.GPIO_PuPd = GPIO_PuPd_UP;		    /* PULL up */
    GPIO_InitType.GPIO_OType = GPIO_OType_PP;	        /* Push-pull mode */
    GPIO_Init(GPIOB, &GPIO_InitType);		          	
    
    //USART_InitStructure.USART_BaudRate = 57600;//ʵ��Ϊ115200
//    USART_InitStructure.USART_BaudRate = 115200;//ʵ��Ϊ230400
//    USART_InitStructure.USART_BaudRate = 288000;//ʵ��Ϊ576000
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
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			     	//���ô����ж�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    ppp_frame_struct.index_frame = 0;//PPP֡�ڲ�ָ��
    ppp_frame_struct.escape_character[0]=0;//PPP֡ת���ַ�������
    ppp_frame_struct.escape_character[1]=0;//PPP֡ת���ַ�������
    ppp_frame_struct.flags_frame=0;//PPP�����ж�
    ppp_frame_struct.length_frame=0;//PPP֡����
		ppp_frame_struct.currt_frame=currt_frame1;
		
    //rs485�շ��������ų�ʼ��
    GPIO_InitType.GPIO_Pin = GPIO_Pin_13;	         
    GPIO_InitType.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitType.GPIO_OType = GPIO_OType_PP;	  		/* �������	*/
    GPIO_InitType.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_Init(GPIOC, &GPIO_InitType);	
    
    RS485_SET_RX_MODE;
}

///*******************************************************************************
//    ��������ppp_rs485_configuration
//    ��  ��: 
//    ��  ��:
//    ����˵�������ڴ��ڵ�PPPͨ��Э��ײ�Ӳ������
//*/
void ppp_rs485_configuration(void)
{ 
    ppp_rs485_configuration_host();
    RS485_SET_RX_MODE;
}

/*******************************************************************************
    ��������CRC_Calculate_PPP
    ��  ��: 
    ��  ��:
    ����˵����CRCУ�������
*/
uint8_t CRC_Calculate_PPP(uint8_t *vector, uint16_t Length)	 /* LengthΪ��Ϣλ���� */
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
    ��������check_ppp_frame
    ��  ��: 
    ��  ��:
    ����˵����У�����PPP֡�Ƿ�����
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
    ��������char_process
    ��  ��: 
    ��  ��:
    ����˵�������ڴ���2��PPPͨ��Э��ײ�Ӳ������ģ��
*/
void ppp_char_process(void)
{
    uint8_t recev_char;
	  uint8_t frame_type;
	  Uart_Queue_Struct temp_Uart_Queue;
    recev_char = USART_ReceiveData(RS485_USART);   //�����Ĵ��������ݻ��浽���ջ�������
    if(recev_char == 0x7E)//����Ŀ�ʼ�����
    {
        if(ppp_frame_struct.index_frame && (check_ppp_frame() == 1))//�յ�������һ֡����
        {
//					  SDA_H;
//					  Delayms(1);
//					  SDA_L;
					
            if(ppp_frame_struct.index_frame>=MAX_FRAME)
						{
								ppp_frame_struct.index_frame=0;
								return ;
						}
            ppp_frame_struct.length_frame = ppp_frame_struct.index_frame-2;//PPP֡���Ȳ�����ͷ��β�������У���ֽ�
            ppp_frame_struct.index_frame=0;//ָ����һ��Ҫ�洢��λ��
						
						switch(ppp_frame_struct.currt_frame) 
						{
							case currt_frame1:
								if((ppp_frame_struct.flags_frame & 0x02) != 0x02)
								{
									ppp_frame_struct.frame1[ppp_frame_struct.index_frame]=recev_char;	
									ppp_frame_struct.currt_frame=currt_frame2;
									ppp_frame_struct.flags_frame = ppp_frame_struct.flags_frame|0x01;//��־֡�յ�����Ҫ�ϲ�Э������ȡ����
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
									ppp_frame_struct.flags_frame = ppp_frame_struct.flags_frame|0x02;//��־֡�յ�����Ҫ�ϲ�Э������ȡ����
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
									ppp_frame_struct.flags_frame = ppp_frame_struct.flags_frame|0x04;//��־֡�յ�����Ҫ�ϲ�Э������ȡ����
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
        else if(ppp_frame_struct.index_frame && check_ppp_frame() == 0)//֡У���,����֡��ͷ����
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
            ppp_frame_struct.index_frame=1;//ָ����һ��Ҫ�洢��λ��		
        }
        else if(ppp_frame_struct.index_frame == 0)//�յ�֡ͷ��־
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
                ppp_frame_struct.index_frame=1;//ָ����һ��Ҫ�洢��λ��		
        }
    }
    else if(recev_char == 0x7D)//�յ�ת���ַ�
    {
        if(ppp_frame_struct.index_frame>0)//��־��һ�ֽ�Ϊת���ַ�
        {
            ppp_frame_struct.escape_character[0]=recev_char;
        }
    }
    else//�յ�֡������
    {
        if(ppp_frame_struct.escape_character[0]==0x7D)//�յ�ת���ַ�
        {
            if(recev_char==0x5E)
            {
                recev_char=0x7E;
            }
            else if(recev_char==0x5D)
            {
                recev_char=0x7D;
            }
            else//δ����ת���ַ�
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
            ppp_frame_struct.index_frame++;//ָ����һ��Ҫ�洢��λ
        }
    }
}

/*******************************************************************************
    ��������ppp_send_frame
    ��  ��: 
    ��  ��:
    ����˵�������ڴ���2��PPPͨ��Э��ײ�Ӳ������֡����
		2018-01-10 �޸�Ϊ�Ƚ���У�飬Ȼ��������
*/
uint16_t ppp_send_frame(uint8_t *source,uint16_t send_length)
{
    uint16_t i;
    uint8_t ppp_crc;
		//����PPP֡У��
    ppp_crc = CRC_Calculate_PPP(source,send_length);
    //��Ӵ򿪷���ģʽ
    RS485_SET_TX_MODE;
    while(USART_GetFlagStatus(RS485_USART, USART_FLAG_TC)==RESET);
    USART_SendData(RS485_USART, 0x7E);	//����PPP֡ͷ
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
//    //����PPP֡У��
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
    //����PPP֡β
    while(USART_GetFlagStatus(RS485_USART, USART_FLAG_TC)==RESET);
    USART_SendData(RS485_USART, 0x7E);	//����PPP֡β
    while(USART_GetFlagStatus(RS485_USART, USART_FLAG_TC)==RESET);
    //��ӹرշ���ģʽ
    RS485_SET_RX_MODE;
    return send_length;
}

/*******************************************************************************
    ��������ppp_send_data
    ��  ��: 
    ��  ��:
    ����˵��:
*/
uint8_t ppp_send_data(uint16_t send_length)
{
	ppp_send_frame(PPP_send_frame,send_length);
	return 0;
}
/*******************************************************************************
    ��������ppp_clear_isr_recv
    ��  ��: 
    ��  ��:
    ����˵��:
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


/*
  
* All rights reserved.
*
* File Name          : rs485.h
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
#ifndef __RS485_H
#define __RS485_H

#include "stm32f4xx.h"

#define RS485_USART USART1 //node\host串口
#define MAX_FRAME 256 //最大帧缓冲长度


struct PPP_FRAME_STRUCT{
    uint16_t index_frame;//PPP帧内部指针
    uint8_t escape_character[2];//PPP帧转义字符分析器
    uint8_t flags_frame;//PPP接收中断
    uint16_t length_frame;//PPP帧长度
    uint8_t frame1[MAX_FRAME];//PPP帧缓冲
	  uint8_t frame2[MAX_FRAME];//PPP帧缓冲
	  uint8_t frame3[MAX_FRAME];//PPP帧缓冲
	  uint8_t currt_frame;
};
typedef struct
{
	uint16_t  length;
	uint8_t * data;
	uint8_t currt_frame;
} Uart_Queue_Struct;

#define currt_frame1 1
#define currt_frame2 2
#define currt_frame3 3
//extern struct PPP_FRAME_STRUCT ppp_frame_struct;
extern uint8_t PPP_send_frame[MAX_FRAME];//PPP发送缓存
#define PPP_send_Dest ((uint8_t *)&PPP_send_frame[0])
#define PPP_send_sour ((uint8_t *)&PPP_send_frame[1])
#define PPP_send_type ((uint8_t *)&PPP_send_frame[2])
#define PPP_data_length ((uint8_t *)&PPP_send_frame[3])
#define PPP_send_data   ((uint8_t *)&PPP_send_frame[4])
#define PPP_recv_frame1 ((uint8_t *)&(ppp_frame_struct.frame1[1]))
#define PPP_recv_frame2 ((uint8_t *)&(ppp_frame_struct.frame2[1]))
#define PPP_recv_frame3 ((uint8_t *)&(ppp_frame_struct.frame3[1]))
	

//外部调用接口
#define RS485_SET_RX_MODE GPIO_ResetBits(GPIOC , GPIO_Pin_13)
#define RS485_SET_TX_MODE GPIO_SetBits(GPIOC , GPIO_Pin_13)

void ppp_rs485_configuration(void);
uint16_t ppp_send_frame(uint8_t *source,uint16_t send_length);
void ppp_char_process(void);
uint8_t CRC_Calculate_PPP(uint8_t *vector , uint16_t Length);
uint8_t ppp_send_data(uint16_t length);
void ppp_clear_isr_recv(Uart_Queue_Struct uart_frame);

#endif /* __RS485_H */


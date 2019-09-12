/*
  
* All rights reserved.
*
* File Name          : uart_eth.c
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
//#include "lwip/opt.h"
//#include "lwip/arch.h"
//#include "lwip/api.h"
//#include "lwip/inet.h"
//#include "lwip/sockets.h"
#include "uart.h"
#include "string.h"
#include "main.h"
#include "rs485.h"
#include "mechanicalcontrol.h"
#include "direction.h"
#include "ads1255.h"
#include "stmflash.h"

/* Scheduler includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define UARTSERVER_THREAD_PRIO    ( tskIDLE_PRIORITY + 5 )

///*******************************************************************************
//    全局结构声明
//*/
int Local_node_num;//定义节点号


void Local_NodeNum_Set_RFlash(void)
{
		u32 R_DATA[2];
		STMFLASH_Read(FLASH_SAVE_NODENUM_ADDR,(u32*)R_DATA,2);
		if( R_DATA[0]!=0x55  )//简单判断
		{
				/*AD默认配置参数*/ 
				Local_node_num = 0x01;
		}
		else 
		{		
					Local_node_num = R_DATA[1];
		}
	  
}
/**
  * @brief uart_server  
  * @param conn: connection socket 
  * @retval None
  */
void uart_server(Uart_Queue_Struct uart_frame) 
{
	uint8_t dest_address,frame_type;//sour_address,length
	//uint8_t adc_start;
  uint8_t adc_prestart;
	uint8_t adc_finish;
	uint8_t data_load;
	portBASE_TYPE xStatus;
	dest_address=uart_frame.data[0];
	//sour_address=uart_frame.data[1];
	frame_type=uart_frame.data[2];
	//length=uart_frame.data[3];
	if((dest_address!=Local_node_num)&&(dest_address!=BRODCAST_ADDRESS))
	{	
    ppp_clear_isr_recv(uart_frame);		
		return;
	}
	*PPP_send_Dest = HOST_ADDRESS;
	*PPP_send_sour = Local_node_num;
	switch(frame_type) 
	{
		case ONLINE_REQUEST:
		  *PPP_send_type = ONLINE_REPLY;
		  *PPP_data_length = 0;
	    ppp_send_data(4);
			break;
		case PARAMETER_CONFIG_REQUEST:
		  *PPP_send_type = PARAMETER_CONFIG_REPLY;
		  *PPP_data_length = 0; 
			sets.sampleFrequency_level = uart_frame.data[4];//freqlevel 0x03
			sets.numOfPoint_level = uart_frame.data[5];//length 0x01
			sets.preSamplingTime_points=uart_frame.data[6];//pre 0x00
			sets.delayTime_ms=uart_frame.data[7];//delay 0x00
			sets.headGain=uart_frame.data[8];//gain 0x01
		  ADC_Parameter_Set();
		  //ad_modules_init()
			ADC_Parameter_Config();	/* Complete all SPI initialization configuration and self-calibration */ 
	    ppp_send_data(4);
			ADC_Parameter_Save_WFLASH();	
			break;
		case CONFIG_ADDRESS_REQUEST:
		  *PPP_send_type = CONFIG_ADDRESS_REPLY;
		  *PPP_data_length = 0; 
			Local_node_num = uart_frame.data[4];
		  *PPP_send_sour = Local_node_num;		
	    ppp_send_data(4);
			ADC_Parameter_Save_WFLASH();	
			break;
		case STRETCH_REQUEST:
			*PPP_send_type = STRETCH_REPLY;
		  *PPP_data_length = 0;
		  mot_stretch();
	    ppp_send_data(4);
			break;
		case APPROACH_REQUEST:
			*PPP_send_type = APPROACH_REPLY;
		  *PPP_data_length = 0;
		  mot_approach();
	    ppp_send_data(4);
			break;
		case LOAD_DIRECTION_REQUEST:
			*PPP_send_type = LOAD_DIRECTION_REPLY;
		  *PPP_data_length = 12;
		  get_direction();
		  memcpy(&PPP_send_data[0],&current_direction[0],12);
	    ppp_send_data(16);
			break;
		case PRE_REQUEST:
			//*PPP_send_type = PRE_REPLY;//预采集不用回复
		  //*PPP_data_length = 0;
		  //SDA_H;
		  adc_prestart=1;
			xQueueSendToBack(xQueue_ADPreStart, &adc_prestart, 0 );
	    //ppp_send_data(4);
			break;
		case SAMPLE_FINISHED_REQUEST:
			*PPP_send_type = SAMPLE_FINISHED_REPLY;
		  *PPP_data_length = 0;
		  xStatus = xQueueReceive( xQueue_ADFinish, &adc_finish, 1000 );//
		  if( xStatus != pdPASS ) return;
	    ppp_send_data(4);
			break;
		case LOAD_DATA_REQUEST:
		  data_load=LOAD_DATA_REQUEST;
		  xQueueSendToBack(xQueue_LoadData, &data_load, 0 );	
			break;
		case LOAD_NOISE_REQUEST:
		  data_load=LOAD_NOISE_REQUEST;
		  xQueueSendToBack(xQueue_LoadData, &data_load, 0 );	
			break;
		case MOT_STATE_REQUEST :
			*PPP_send_type = MOT_STATE_REPLY;
		  *PPP_data_length = 1;
		  *PPP_send_data = mot_run_state;
	    ppp_send_data(5);
			break;
		case MOT_OFF_REQUEST :
			*PPP_send_type = MOT_OFF_REPLY;
		  *PPP_data_length = 0;
		  mot_power_off;
		  mot_run_state=Mot_stoped;
	    ppp_send_data(4);
			break;
		case TEST_GAIN_REQUEST :
		  data_load=TEST_GAIN_REQUEST;
		  xQueueSendToBack(xQueue_LoadData, &data_load, 0 );	
			break;
		case TEST_WITHIN_NOISE_REQUEST :
		  data_load=TEST_WITHIN_NOISE_REQUEST;
		  xQueueSendToBack(xQueue_LoadData, &data_load, 0 );	
			break;
		case TEST_IMPULSE_RESPONSE_REQUEST :
		  data_load=TEST_IMPULSE_RESPONSE_REQUEST;
		  xQueueSendToBack(xQueue_LoadData, &data_load, 0 );	
			break;
		case TEST_CONFORMANCE_REQUEST :
		  data_load=TEST_CONFORMANCE_REQUEST;
		  xQueueSendToBack(xQueue_LoadData, &data_load, 0 );	
			break;
		case TSET_CROSSTALK_REQUEST :
		  data_load=TSET_CROSSTALK_REQUEST;
		  xQueueSendToBack(xQueue_LoadData, &data_load, 0 );	
			break;
		case TEST_COMMON_MODE_REQUEST :
		  data_load=TEST_COMMON_MODE_REQUEST;
		  xQueueSendToBack(xQueue_LoadData, &data_load, 0 );	
			break;
		case TEST_DISTORTION_REQUEST :
		  data_load=TEST_DISTORTION_REQUEST;
		  xQueueSendToBack(xQueue_LoadData, &data_load, 0 );	
			break;
		case TEST_RESISTANCE_REQUEST:
			data_load=TEST_RESISTANCE_REQUEST;
			xQueueSendToBack(xQueue_LoadData, &data_load, 0 );	
		break;
		//添加传感器检测模块
		case TEST_WITHIN_NOISE_SENSOR_REQUEST:
			data_load=TEST_WITHIN_NOISE_SENSOR_REQUEST;
			xQueueSendToBack(xQueue_LoadData, &data_load, 0 );	
			break;
		case TEST_DISTORTION_SENSOR_REQUEST:
			data_load=TEST_DISTORTION_SENSOR_REQUEST;
			xQueueSendToBack(xQueue_LoadData, &data_load, 0 );	
			break;
		case TEST_COMMON_MODE_SENSOR_REQUEST:
			data_load=TEST_COMMON_MODE_SENSOR_REQUEST;
			xQueueSendToBack(xQueue_LoadData, &data_load, 0 );	
			break;
		case TEST_IMPULSE_RESPONSE_SENSOR_REQUEST:
			data_load=TEST_IMPULSE_RESPONSE_SENSOR_REQUEST;
			xQueueSendToBack(xQueue_LoadData, &data_load, 0 );	
			break;
		case TEST_TILT_SENSOR_REQUEST:
			data_load=TEST_TILT_SENSOR_REQUEST;
			xQueueSendToBack(xQueue_LoadData, &data_load, 0 );	
			break;
		case TEST_GRAVITY_SENSOR_REQUEST:
			data_load=TEST_GRAVITY_SENSOR_REQUEST;
			xQueueSendToBack(xQueue_LoadData, &data_load, 0 );	
			break;
		//添加传感器检测模块结束
		//添加电子罗盘校准部分
		case SET_DIRECTION_UP_REQUEST://垂直安装
			*PPP_send_type = SET_DIRECTION_UP_REPLY;
		  *PPP_data_length = 0;
		  IICwriteBytes(0x50,DIRECTION_VER[0],2,&DIRECTION_VER[1]);//发送函数
		  ppp_send_data(4);
			break;
		case SET_DIRECTION_HOR_REQUEST://水平安装
			*PPP_send_type = SET_DIRECTION_HOR_REPLY;
		  *PPP_data_length = 0;
		  IICwriteBytes(0x50,DIRECTION_HOR[0],2,&DIRECTION_HOR[1]);//发送函数
		  ppp_send_data(4);
			break;
		case SET_ALGORITHM_REQUEST://alxis9 算法
			*PPP_send_type = SET_ALGORITHM_REPLY;
		  *PPP_data_length = 0;
		  IICwriteBytes(0x50,ALG9[0],2,&ALG9[1]);//发送函数
		  ppp_send_data(4);
			break;		
		case SET_ALGORITHM_6_REQUEST://alxis6 算法
			*PPP_send_type = SET_ALGORITHM_REPLY;
		  *PPP_data_length = 0;
		  IICwriteBytes(0x50,ALG6[0],2,&ALG6[1]);//发送函数
		  ppp_send_data(4);
			break;
		case SET_GYRO_RANGE_REQUEST:
			*PPP_send_type = SET_GYRO_RANGE_REPLY;
		  *PPP_data_length = 0;
		  IICwriteBytes(0x50,GYRORANGE[0],2,&GYRORANGE[1]);//发送函数
		  ppp_send_data(4);
		break;		
		case SET_GYRO_500_RANGE_REQUEST:
			*PPP_send_type = SET_GYRO_RANGE_REPLY;
		  *PPP_data_length = 0;
		  IICwriteBytes(0x50,GYRORANGE_500[0],2,&GYRORANGE_500[1]);//发送函数
		  ppp_send_data(4);
		break;		
		case SET_GYRO_1000_RANGE_REQUEST:
			*PPP_send_type = SET_GYRO_RANGE_REPLY;
		  *PPP_data_length = 0;
		  IICwriteBytes(0x50,GYRORANGE_1000[0],2,&GYRORANGE_1000[1]);//发送函数
		  ppp_send_data(4);
		break;
		case SET_GYRO_2000_RANGE_REQUEST:
			*PPP_send_type = SET_GYRO_RANGE_REPLY;
		  *PPP_data_length = 0;
		  IICwriteBytes(0x50,GYRORANGE_2000[0],2,&GYRORANGE_2000[1]);//发送函数
		  ppp_send_data(4);
		break;		
		case SET_GYRO_BANDWIDTH_REQUEST:
			*PPP_send_type = SET_GYRO_BANDWIDTH_REPLY;
		  *PPP_data_length = 0;
		  IICwriteBytes(0x50,BANDWIDTH[0],2,&BANDWIDTH[1]);//发送函数
		  ppp_send_data(4);
			break;		
		case SET_GYRO_256_BANDWIDTH_REQUEST:
			*PPP_send_type = SET_GYRO_BANDWIDTH_REPLY;
		  *PPP_data_length = 0;
		  IICwriteBytes(0x50,BANDWIDTH_256[0],2,&BANDWIDTH_256[1]);//发送函数
		  ppp_send_data(4);
		break;
		case SET_GYRO_188_BANDWIDTH_REQUEST:
			*PPP_send_type = SET_GYRO_BANDWIDTH_REPLY;
		  *PPP_data_length = 0;
		  IICwriteBytes(0x50,BANDWIDTH_188[0],2,&BANDWIDTH_188[1]);//发送函数
		  ppp_send_data(4);
		break;
		case SET_GYRO_98_BANDWIDTH_REQUEST:
			*PPP_send_type = SET_GYRO_BANDWIDTH_REPLY;
		  *PPP_data_length = 0;
		  IICwriteBytes(0x50,BANDWIDTH_98[0],2,&BANDWIDTH_98[1]);//发送函数
		  ppp_send_data(4);
		break;
		case SET_GYRO_42_BANDWIDTH_REQUEST:
			*PPP_send_type = SET_GYRO_BANDWIDTH_REPLY;
		  *PPP_data_length = 0;
		  IICwriteBytes(0x50,BANDWIDTH_42[0],2,&BANDWIDTH_42[1]);//发送函数
		  ppp_send_data(4);
		break;
		case SET_GYRO_10_BANDWIDTH_REQUEST:
			*PPP_send_type = SET_GYRO_BANDWIDTH_REPLY;
		  *PPP_data_length = 0;
		  IICwriteBytes(0x50,BANDWIDTH_10[0],2,&BANDWIDTH_10[1]);//发送函数
		  ppp_send_data(4);
		break;
		case SET_GYRO_5_BANDWIDTH_REQUEST:
			*PPP_send_type = SET_GYRO_BANDWIDTH_REPLY;
		  *PPP_data_length = 0;
		  IICwriteBytes(0x50,BANDWIDTH_5[0],2,&BANDWIDTH_5[1]);//发送函数
		  ppp_send_data(4);
		break;		
		case SET_GYRO_ACCELERATION_RANGE_REQUEST:
			*PPP_send_type = SET_GYRO_ACCELERATION_RANGE_REPLY;
		  *PPP_data_length = 0;
		  IICwriteBytes(0x50,ACCRANGE[0],2,&ACCRANGE[1]);//发送函数
		  ppp_send_data(4);
			break;		
		case SET_GYRO_ACCELERATION_4_RANGE_REQUEST:
			*PPP_send_type = SET_GYRO_ACCELERATION_RANGE_REPLY;
		  *PPP_data_length = 0;
		  IICwriteBytes(0x50,ACCRANGE_4g[0],2,&ACCRANGE_4g[1]);//发送函数
		  ppp_send_data(4);
			break;
		case SET_GYRO_ACCELERATION_8_RANGE_REQUEST:
			*PPP_send_type = SET_GYRO_ACCELERATION_RANGE_REPLY;
		  *PPP_data_length = 0;
		  IICwriteBytes(0x50,ACCRANGE_8g[0],2,&ACCRANGE_8g[1]);//发送函数
		  ppp_send_data(4);
			break;		
		case SET_GYRO_ACCELERATION_16_RANGE_REQUEST:
			*PPP_send_type = SET_GYRO_ACCELERATION_RANGE_REPLY;
		  *PPP_data_length = 0;
		  IICwriteBytes(0x50,ACCRANGE_16g[0],2,&ACCRANGE_16g[1]);//发送函数
		  ppp_send_data(4);
			break;
				
		case READADDR_REQUEST://读取JY901配置
			*PPP_send_type = READADDR_REQUEST_REPLY;
		  *PPP_data_length = 10;
			IICreadBytes(0x50, 0x23, 2,&direction_data[0]);//读取安装方向
			IICreadBytes(0x50, 0x24, 2,&direction_data[2]);//读取算法	
			IICreadBytes(0x50, 0x21, 2,&direction_data[4]);//读取加速度范围
			IICreadBytes(0x50, 0x20, 2,&direction_data[6]);//读取陀螺仪范围			
			IICreadBytes(0x50, 0x1f, 2,&direction_data[8]);//读取带宽				
			memcpy(&PPP_send_data[0],&direction_data[0],10);
		  ppp_send_data(14);
			break;					
		case RESET_DIRECTION_CONFIG_REQUEST://恢复默认设置
			*PPP_send_type = RESET_DIRECTION_CONFIG_REPLY;
		  *PPP_data_length = 0;
		  IICwriteBytes(0x50,RESET_CONFIG[0],2,&RESET_CONFIG[1]);//发送函数
		  ppp_send_data(4);
			break;
//		case ACC_CALIBRATION_REQUEST://进入加速度校准模式
//			*PPP_send_type = ACC_CALIBRATION_REPLY;
//		  *PPP_data_length = 0;
//		  IICwriteBytes(0x50,ACC_CALSW[0],2,&ACC_CALSW[1]);//发送函数
//		  ppp_send_data(4);
//			break;
		case ACC_START_CALIBRATION_REQUEST://开始校准
			*PPP_send_type = ACC_START_CALIBRATION_REPLY;
		  *PPP_data_length = 0;
//			IICwriteBytes(0x50,ACC_CALSW[0],2,&ACC_CALSW[1]);//发送函数
//		  Delayms(100);
			IICwriteBytes(0x50,AXOFF[0],2,&AXOFF[1]);//发送函数，命令之间需要延时100ms
			DELAY_MS(100);
			IICwriteBytes(0x50,AYOFF[0],2,&AXOFF[1]);//发送函数
			DELAY_MS(100);
			IICwriteBytes(0x50,AZOFF[0],2,&AXOFF[1]);//发送函数
		  ppp_send_data(4);
			break;
		case ACC_UP_DATA_REQUEST:
			*PPP_send_type = ACC_UP_DATA_REPLY;
		  *PPP_data_length = 6;
			IICreadBytes(0x50, 0x34, 2,&direction_data[0]);//X轴加速度
			IICreadBytes(0x50, 0x35, 2,&direction_data[2]);//Y轴加速度
			IICreadBytes(0x50, 0x36, 2,&direction_data[4]);//Z轴加速度
			memcpy(&PPP_send_data[0],&direction_data[0],6);
		 ppp_send_data(10);
			break;
		case ACC_WRITE_PARAMETER_REQUEST:
			*PPP_send_type = ACC_WRITE_PARAMETER_REPLY;
		  *PPP_data_length = 0; 
			direction_data[0]=uart_frame.data[4];//AXL
			direction_data[1]=uart_frame.data[5];//AXH
			direction_data[2]=uart_frame.data[6];//AYL
			direction_data[3]=uart_frame.data[7];//AYH
			direction_data[4]=uart_frame.data[8];//AZL
			direction_data[5]=uart_frame.data[9];//AZH	
//			IICwriteBytes(0x50,CALSW_QUIT[0],2,&CALSW_QUIT[1]);//退出校准模式			
//			Delayms(100);
			IICwriteBytes(0x50,AXOFF[0],2,&direction_data[0]);//发送函数，命令之间需要延时100ms
			DELAY_MS(100);
			IICwriteBytes(0x50,AYOFF[0],2,&direction_data[2]);//发送函数
			DELAY_MS(100);
			IICwriteBytes(0x50,AZOFF[0],2,&direction_data[4]);//发送函数
			DELAY_MS(100);
			IICwriteBytes(0x50,SAVE_CONFIG[0],2,&SAVE_CONFIG[1]);//保存配置
			ppp_send_data(4);
		break;
//		case MAG_CALIBRATION_REQUEST:
//			*PPP_send_type = MAG_CALIBRATION_REPLY;
//		  *PPP_data_length = 0;
//			IICwriteBytes(0x50,MAG_CALSW[0],2,&MAG_CALSW[1]);//发送函数
//		  ppp_send_data(4);
//		break;
		case MAG_START_CALIBRATION_REQUEST:
			*PPP_send_type = MAG_START_CALIBRATION_REPLY;
		  *PPP_data_length = 0;
		  IICwriteBytes(0x50,HXOFF[0],2,&HXOFF[1]);//发送函数，命令之间需要延时100ms
			DELAY_MS(100);
			IICwriteBytes(0x50,HYOFF[0],2,&HYOFF[1]);//发送函数
			DELAY_MS(100);
			IICwriteBytes(0x50,HZOFF[0],2,&HZOFF[1]);//发送函数
		  ppp_send_data(4);
			break;
		case MAG_UP_DATA_REQUEST:
			*PPP_send_type = MAG_UP_DATA_REPLY;
		  *PPP_data_length = 6;
			IICreadBytes(0x50, 0x3a, 2,&direction_data[0]);//X轴磁场
			IICreadBytes(0x50, 0x3b, 2,&direction_data[2]);//Y轴磁场
			IICreadBytes(0x50, 0x3c, 2,&direction_data[4]);//Z轴磁场
			memcpy(&PPP_send_data[0],&direction_data[0],6);
		 ppp_send_data(10);
			break;
		case MAG_WRITE_PARAMETER_REQUEST:
			*PPP_send_type = MAG_WRITE_PARAMETER_REQUEST_REPLY;
		  *PPP_data_length = 0; 
			direction_data[0]=uart_frame.data[4];//HXL
			direction_data[1]=uart_frame.data[5];//HXH
			direction_data[2]=uart_frame.data[6];//HYL
			direction_data[3]=uart_frame.data[7];//HYH
			direction_data[4]=uart_frame.data[8];//HZL
			direction_data[5]=uart_frame.data[9];//HZH	

			IICwriteBytes(0x50,HXOFF[0],2,&direction_data[0]);//发送函数，命令之间需要延时100ms
			DELAY_MS(100);
			IICwriteBytes(0x50,HYOFF[0],2,&direction_data[2]);//发送函数
			DELAY_MS(100);
			IICwriteBytes(0x50,HZOFF[0],2,&direction_data[4]);//发送函数
			DELAY_MS(100);
			IICwriteBytes(0x50,SAVE_CONFIG[0],2,&SAVE_CONFIG[1]);//保存配置
			ppp_send_data(4);
		break;
		
		case GYRO_UP_DATAS_REQUEST:
			*PPP_send_type = GYRO_UP_DATAS_REPLY;
		  *PPP_data_length =24;
			IICreadBytes(0x50, 0x34, 2,&direction_data[0]);//X轴加速度
			IICreadBytes(0x50, 0x35, 2,&direction_data[2]);//Y轴加速度
			IICreadBytes(0x50, 0x36, 2,&direction_data[4]);//Z轴加速度
			IICreadBytes(0x50, 0x37, 2,&direction_data[6]);//X轴角速度
			IICreadBytes(0x50, 0x38, 2,&direction_data[8]);//Y轴角速度
			IICreadBytes(0x50, 0x39, 2,&direction_data[10]);//Z轴角速度
			IICreadBytes(0x50, 0x3a, 2,&direction_data[12]);//X轴磁场
			IICreadBytes(0x50, 0x3b, 2,&direction_data[14]);//Y轴磁场
			IICreadBytes(0x50, 0x3c, 2,&direction_data[16]);//Z轴磁场
			IICreadBytes(0x50, 0x3d, 2,&direction_data[18]);//X轴角度
			IICreadBytes(0x50, 0x3e, 2,&direction_data[20]);//Y轴角度
			IICreadBytes(0x50, 0x3f, 2,&direction_data[22]);//Z轴角度
			memcpy(&PPP_send_data[0],&direction_data[0],24);
			ppp_send_data(28);
			break;	
//添加陀螺仪校准模块结束		
		
		default:
			*PPP_send_type = ONLINE_REPLY;
		  *PPP_data_length = 0;
	    ppp_send_data(4);
      break;			
	}
	ppp_clear_isr_recv(uart_frame);
}

/**
  * @brief  uart thread 
  * @param arg: pointer on argument(not used here) 
  * @retval None
  */
static void uart_thread(void *arg)
{
	Uart_Queue_Struct uart_frame;
	portBASE_TYPE xStatus;
	Local_NodeNum_Set_RFlash();
  while (1) 
  {
		xStatus = xQueueReceive( xQueue_Uart, &uart_frame, portMAX_DELAY );//INCLUDE_vTaskSuspend?
    if( xStatus == pdPASS )
		{
			uart_server(uart_frame);
		}
  }
}
//    函数名：uart_init
//    输  入: 
//    输  出:
//    功能说明：
//*/
void uart_init()
{
	if( xQueue_Uart != NULL )
	{
		xTaskCreate(uart_thread, (int8_t *) "Urt/Eth", configMINIMAL_STACK_SIZE * 1, NULL,UARTSERVER_THREAD_PRIO, NULL);
	}
}

   


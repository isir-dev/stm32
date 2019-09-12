/*
  
* All rights reserved.
*
* File Name          : sensor_test.c
* Description        : 利用片内DAC产生标准信号对传感器状态及各传感器一致性进行测定
*
* Version            : V1.0
* Author             : liu Yong
* Date               : 7/7/2012
*
* OriginalVersion    : V1.0
* Original Author    : liu Yong
* Date               : 7/7/2012
*/

#include "sensor_test.h"
#include "SignalsGeneration.h"



void momal_sample_gpio_init(void)
{
	  #ifdef Stack_X
    SwitchToSampleSig_X;
	  #endif
		#ifdef Stack_Y
    SwitchToSampleSig_Y;
	  #endif
		#ifdef Stack_Z
    SwitchToSampleSig_Z;
	  #endif
}



/*******************************************************************************
    函数名：test_gain
    输  入: 
    输  出:
    功能说明：增益精度测试：测标准正弦信号
*/
void test_gain(void)
{
    //产生30HZ 1Vpp标准正弦信号
    signals_ON(signal_sina_30HZ);
    //切换信号输入通道为测试通道
		#ifdef Stack_X
    SwitchToSTDSigTest_X;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Y
    SwitchToSTDSigTest_Y;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Z
    SwitchToSTDSigTest_Z;/* 模拟开关切换至测试信号 */
    #endif
    Delay_us(500);
    //采集2048个点
		AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);

    //切换信号输入通道为传感器信号
		#ifdef Stack_X
    SwitchToSampleSig_X;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Y
    SwitchToSampleSig_Y;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Z
    SwitchToSampleSig_Z;/* 模拟开关切换至测试信号 */
    #endif
    //关闭标准信号源以防止干扰
    signals_OFF();
    
}

/*******************************************************************************
    函数名：test_gain
    输  入: 
    输  出:
    功能说明：增益精度测试：方波测试
		2017-09-12
*/
void test_gain_Pulsewave(void)
{
    //产生2HZ 50% 8+39+465方波
    signals_ON(Pulsewave12bit_2HZ_50);
    //切换信号输入通道为测试通道
		#ifdef Stack_X
    SwitchToSTDSigTest_X;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Y
    SwitchToSTDSigTest_Y;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Z
    SwitchToSTDSigTest_Z;/* 模拟开关切换至测试信号 */
    #endif
	  ADC_Parameter_Config_Selftest_start();
//    Delay_us(50);
    //采集2048个点
		AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);

    //切换信号输入通道为传感器信号
		#ifdef Stack_X
    SwitchToSampleSig_X;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Y
    SwitchToSampleSig_Y;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Z
    SwitchToSampleSig_Z;/* 模拟开关切换至测试信号 */
    #endif
		ADC_Parameter_Config();
    //关闭标准信号源以防止干扰
    signals_OFF();
}

/*******************************************************************************
    函数名：background_noise
    输  入: 
    输  出:
    功能说明：背景噪声监控
*/
void background_noise(void)
{
	  AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);
}

/*******************************************************************************
    函数名：test_within_noise
    输  入: 
    输  出:
    功能说明：内噪声测试
*/
void test_within_noise(void)
{

	  ADS1255_RESET();
	  ADC_Parameter_Config();
    Delay_us(1000);//延时，等待信号源稳定
    //采集2048个点
	  AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);
    
}
/*******************************************************************************
    函数名：test_impulse_response
    输  入: 
    输  出:
    功能说明：脉冲响应测试：测标准方波
*/
void test_impulse_response(void)
{
    
    signals_ON(Pulsewave12bit_2HZ_75);
    //切换信号输入通道为测试通道
	  #ifdef Stack_X
    SwitchToSTDSigTest_X;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Y
    SwitchToSTDSigTest_Y;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Z
    SwitchToSTDSigTest_Z;/* 模拟开关切换至测试信号 */
    #endif
	  ADC_Parameter_Config_Selftest_start();
    Delay_us(50);
    //采集2048个点
	  AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);
    //切换信号输入通道为传感器信号
	  #ifdef Stack_X
    SwitchToSampleSig_X;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Y
    SwitchToSampleSig_Y;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Z
    SwitchToSampleSig_Z;/* 模拟开关切换至测试信号 */
    #endif
		ADC_Parameter_Config();
    //关闭标准信号源以防止干扰
    signals_OFF();
    
}
/*******************************************************************************
    函数名：test_conformance
    输  入: 
    输  出:
    功能说明：一致性测试：测标准正弦信号
*/
void test_conformance(void)
{
    //产生30HZ 1Vpp标准正弦信号
    signals_ON(signal_sina_30HZ);
    //切换信号输入通道为测试通道
    #ifdef Stack_X
    SwitchToSTDSigTest_X;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Y
    SwitchToSTDSigTest_Y;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Z
    SwitchToSTDSigTest_Z;/* 模拟开关切换至测试信号 */
    #endif
	  ADC_Parameter_Config_Selftest_start();
    Delay_us(50);
    //采集2048个点
	  AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);
    //切换信号输入通道为传感器信号
    #ifdef Stack_X
    SwitchToSampleSig_X;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Y
    SwitchToSampleSig_Y;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Z
    SwitchToSampleSig_Z;/* 模拟开关切换至测试信号 */
    #endif
		ADC_Parameter_Config();
    //关闭标准信号源以防止干扰
    signals_OFF();
}
/*******************************************************************************
    函数名：test_crosstalk
    输  入: 
    输  出:
    功能说明：串音测试：：测标准正弦信号
*/
void test_crosstalk(void)
{
    //产生30HZ 1Vpp标准正弦信号
    signals_ON(signal_sina_30HZ);
    //切换信号输入通道为测试通道
	  #ifdef Stack_X
    SwitchToSTDSigTest_X;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Y
    SwitchToSampleSig_Y;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Z
    SwitchToSampleSig_Y;/* 模拟开关切换至测试信号 */
    #endif
	  ADC_Parameter_Config_Selftest_start();
    Delay_us(50);
    //采集2048个点
	  AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);
    //切换信号输入通道为传感器信号
    #ifdef Stack_X
    SwitchToSampleSig_X;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Y
    SwitchToSampleSig_Y;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Z
    SwitchToSampleSig_Z;/* 模拟开关切换至测试信号 */
    #endif
		ADC_Parameter_Config();
    //关闭标准信号源以防止干扰
    signals_OFF();
    
    
}
/*******************************************************************************
    函数名：test_common_mode
    输  入: 
    输  出:
    功能说明：共模测试：：测标准正弦信号
*/
void test_common_mode(void)
{//暂不支持
    //产生30HZ 1Vpp标准正弦信号
    signals_ON(signal_sina_7_8125HZ_776);
    //切换信号输入通道为测试通道
    #ifdef Stack_X
    SwitchToSTDSigTest_X;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Y
    SwitchToSTDSigTest_Y;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Z
    SwitchToSTDSigTest_Z;/* 模拟开关切换至测试信号 */
    #endif
	  ADC_Parameter_Config_Selftest_start();
    Delay_us(50);
    
    //采集2048个点
	  AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);
    //切换信号输入通道为传感器信号
    #ifdef Stack_X
    SwitchToSampleSig_X;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Y
    SwitchToSampleSig_Y;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Z
    SwitchToSampleSig_Z;/* 模拟开关切换至测试信号 */
    #endif
		ADC_Parameter_Config();
    //关闭标准信号源以防止干扰
    signals_OFF();    
}
/*******************************************************************************
    函数名：test_distortion
    输  入: 
    输  出:
    功能说明：畸变测试：：测标准正弦信号
*/
void test_distortion(void)
{//暂不支持
    //产生30HZ 1Vpp标准正弦信号
    signals_ON(signal_sina_7_8125HZ_97);
    //切换信号输入通道为测试通道
    #ifdef Stack_X
    SwitchToSTDSigTest_X;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Y
    SwitchToSTDSigTest_Y;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Z
    SwitchToSTDSigTest_Z;/* 模拟开关切换至测试信号 */
    #endif
	  ADC_Parameter_Config_Selftest_start();
    Delay_us(50);
    //采集2048个点
	  AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);
    //切换信号输入通道为传感器信号
		#ifdef Stack_X
    SwitchToSampleSig_X;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Y
    SwitchToSampleSig_Y;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Z
    SwitchToSampleSig_Z;/* 模拟开关切换至测试信号 */
    #endif
		ADC_Parameter_Config();
    //关闭标准信号源以防止干扰
    signals_OFF();
}

/*******************************************************************************
    函数名：test_distortion
    输  入: 
    输  出:
    功能说明：电阻测试 
		2017-09-12
*/
void test_res(void)
{//暂不支持
    //产生7.8125HZ 97%标准正弦信号
    signals_ON(signal_sina_7_8125HZ_776);
    //切换信号输入通道为测试通道
    #ifdef Stack_X
    SwitchToSTDSigTest_X;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Y
    SwitchToSTDSigTest_Y;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Z
    SwitchToSTDSigTest_Z;/* 模拟开关切换至测试信号 */
    #endif
	  ADC_Parameter_Config_Selftest_start();
    Delay_us(50);
    //采集2048个点
	  AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);
    //切换信号输入通道为传感器信号
		#ifdef Stack_X
    SwitchToSampleSig_X;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Y
    SwitchToSampleSig_Y;/* 模拟开关切换至测试信号 */
    #endif
		#ifdef Stack_Z
    SwitchToSampleSig_Z;/* 模拟开关切换至测试信号 */
    #endif
		ADC_Parameter_Config();
    //关闭标准信号源以防止干扰
    signals_OFF();
}

//void auto_test(void)
//{
//    //测试DA输出
//    int i=0;
//    DAC_Cmd(DAC_Channel_1, ENABLE);
//    while(1)
//    {
//        DAC_SetChannel1Data(DAC_Align_12b_R,i);		/* 12位右对齐，三角波偏置为0 */
//        //StartTimer(DAC_TIMER,2);
//        //while(CheckTimer(DAC_TIMER)!= 1)
//        Delay_us(100);//周期约10ms，幅值0-1024对应0-1v
//        i++;
//        if(i>1023)
//            i=0;
//    }
//}

/*******************************************************************************************************
传感器测试部分
方案一DA不断输出信号，AD对传感器输出信号和DA信号叠加采集
********************************************************************************************************/


/*******************************************************************************
    函数名：sensor_test_within_noise
    输  入: 
    输  出:
    功能说明：传感器噪声测试，直接采集传感器静态时的数据
		2017-09-12
*/
void sensor_test_within_noise(void)
{
	  Delay_us(50);
		AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);
}


/*******************************************************************************
    函数名：sensor_test_common_mode
    输  入: 
    输  出:
    功能说明：传感器共模测试 ：正弦信号
		2017-09-12
*/
void sensor_test_common_mode(void)
{
	//打开测试通道开关（光耦）
	SENSOR_CHECK_ON;
	//产生31.25hz 77.6%振幅正弦信号
	signals_ON(signal_sina_30HZ);
	#ifdef Stack_X
	SwitchToSTDSigTest_X;/* 模拟开关切换至测试信号 */
	#endif
	#ifdef Stack_Y
	SwitchToSTDSigTest_Y;/* 模拟开关切换至测试信号 */
	#endif
	#ifdef Stack_Z
	SwitchToSTDSigTest_Z;/* 模拟开关切换至测试信号 */
	#endif
	ADC_Parameter_Config_Selftest_start();
	Delay_us(50);
	AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);
	//关闭标准信号源以防止干扰
  signals_OFF();
	//关闭测试通道
	SENSOR_CHECK_OFF;
	 //切换信号输入通道为传感器信号
	#ifdef Stack_X
	SwitchToSampleSig_X;/* 模拟开关切换至传感器输出信号 */
	#endif
	#ifdef Stack_Y
	SwitchToSampleSig_Y;/* 模拟开关切换至传感器输出信号 */
	#endif
	#ifdef Stack_Z
	SwitchToSampleSig_Z;/* 模拟开关切换至传感器输出信号 */
	#endif
	ADC_Parameter_Config();
}
/*******************************************************************************
    函数名：sensor_test_impulse_response
    输  入: 
    输  出:
    功能说明：传感器脉冲相应测试，测试脉冲信号
		2017-09-13
*/
void sensor_test_impulse_response(void)
{
	//打开测试通道开关（光耦）
	SENSOR_CHECK_ON;
	signals_ON(Pulsewave12bit_2HZ_75);
	#ifdef Stack_X
	SwitchToSTDSigTest_X;/* 模拟开关切换至测试信号 */
	#endif
	#ifdef Stack_Y
	SwitchToSTDSigTest_Y;/* 模拟开关切换至测试信号 */
	#endif
	#ifdef Stack_Z
	SwitchToSTDSigTest_Z;/* 模拟开关切换至测试信号 */
	#endif
	ADC_Parameter_Config_Selftest_start();
	Delay_us(50);
	AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);
	//关闭标准信号源以防止干扰
  signals_OFF();
	//关闭测试通道
	SENSOR_CHECK_OFF;
	//切换信号输入通道为传感器信号
	#ifdef Stack_X
	SwitchToSampleSig_X;/* 模拟开关切换至传感器输出信号 */
	#endif
	#ifdef Stack_Y
	SwitchToSampleSig_Y;/* 模拟开关切换至传感器输出信号 */
	#endif
	#ifdef Stack_Z
	SwitchToSampleSig_Z;/* 模拟开关切换至传感器输出信号 */
	#endif
	ADC_Parameter_Config();
	
}

/*******************************************************************************
    函数名：sensor_test_distortion
    输  入: 
    输  出:
    功能说明：传感器畸变测试
		2017-09-13
*/
void sensor_test_distortion(void)
{
	//打开测试通道开关（光耦）
	SENSOR_CHECK_ON;
	signals_ON(signal_sina_7_8125HZ_97);
	#ifdef Stack_X
	SwitchToSTDSigTest_X;/* 模拟开关切换至测试信号 */
	#endif
	#ifdef Stack_Y
	SwitchToSTDSigTest_Y;/* 模拟开关切换至测试信号 */
	#endif
	#ifdef Stack_Z
	SwitchToSTDSigTest_Z;/* 模拟开关切换至测试信号 */
	#endif
	ADC_Parameter_Config_Selftest_start();
	Delay_us(50);
	AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);
	//关闭标准信号源以防止干扰
  signals_OFF();
	//关闭测试通道
	SENSOR_CHECK_OFF;
	//切换信号输入通道为传感器信号
	#ifdef Stack_X
	SwitchToSampleSig_X;/* 模拟开关切换至传感器输出信号 */
	#endif
	#ifdef Stack_Y
	SwitchToSampleSig_Y;/* 模拟开关切换至传感器输出信号 */
	#endif
	#ifdef Stack_Z
	SwitchToSampleSig_Z;/* 模拟开关切换至传感器输出信号 */
	#endif
	ADC_Parameter_Config();
}

/*******************************************************************************
    函数名：sensor_test_titl
    输  入: 
    输  出:
    功能说明：传感器倾斜测试
		2017-09-13
*/

void sensor_test_tilt(void)
{
	Delay_us(50);
	AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);
}

/*******************************************************************************
    函数名：sensor_test_distortion
    输  入: 
    输  出:
    功能说明：传感器重力测试
		2017-09-13
*/

void sensor_test_gravity(void)
{

	Delay_us(50);
	AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);
}

//void auto_test(void)
//{
//    //测试DA输出
//    int i=0;
//    DAC_Cmd(DAC_Channel_1, ENABLE);
//    while(1)
//    {
//        DAC_SetChannel1Data(DAC_Align_12b_R,i);		/* 12位右对齐，三角波偏置为0 */
//        //StartTimer(DAC_TIMER,2);
//        //while(CheckTimer(DAC_TIMER)!= 1)
//        Delay_us(100);//周期约10ms，幅值0-1024对应0-1v
//        i++;
//        if(i>1023)
//            i=0;
//    }
//}

/*
  
* All rights reserved.
*
* File Name          : sensor_test.c
* Description        : ����Ƭ��DAC������׼�źŶԴ�����״̬����������һ���Խ��вⶨ
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
    ��������test_gain
    ��  ��: 
    ��  ��:
    ����˵�������澫�Ȳ��ԣ����׼�����ź�
*/
void test_gain(void)
{
    //����30HZ 1Vpp��׼�����ź�
    signals_ON(signal_sina_30HZ);
    //�л��ź�����ͨ��Ϊ����ͨ��
		#ifdef Stack_X
    SwitchToSTDSigTest_X;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Y
    SwitchToSTDSigTest_Y;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Z
    SwitchToSTDSigTest_Z;/* ģ�⿪���л��������ź� */
    #endif
    Delay_us(500);
    //�ɼ�2048����
		AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);

    //�л��ź�����ͨ��Ϊ�������ź�
		#ifdef Stack_X
    SwitchToSampleSig_X;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Y
    SwitchToSampleSig_Y;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Z
    SwitchToSampleSig_Z;/* ģ�⿪���л��������ź� */
    #endif
    //�رձ�׼�ź�Դ�Է�ֹ����
    signals_OFF();
    
}

/*******************************************************************************
    ��������test_gain
    ��  ��: 
    ��  ��:
    ����˵�������澫�Ȳ��ԣ���������
		2017-09-12
*/
void test_gain_Pulsewave(void)
{
    //����2HZ 50% 8+39+465����
    signals_ON(Pulsewave12bit_2HZ_50);
    //�л��ź�����ͨ��Ϊ����ͨ��
		#ifdef Stack_X
    SwitchToSTDSigTest_X;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Y
    SwitchToSTDSigTest_Y;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Z
    SwitchToSTDSigTest_Z;/* ģ�⿪���л��������ź� */
    #endif
	  ADC_Parameter_Config_Selftest_start();
//    Delay_us(50);
    //�ɼ�2048����
		AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);

    //�л��ź�����ͨ��Ϊ�������ź�
		#ifdef Stack_X
    SwitchToSampleSig_X;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Y
    SwitchToSampleSig_Y;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Z
    SwitchToSampleSig_Z;/* ģ�⿪���л��������ź� */
    #endif
		ADC_Parameter_Config();
    //�رձ�׼�ź�Դ�Է�ֹ����
    signals_OFF();
}

/*******************************************************************************
    ��������background_noise
    ��  ��: 
    ��  ��:
    ����˵���������������
*/
void background_noise(void)
{
	  AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);
}

/*******************************************************************************
    ��������test_within_noise
    ��  ��: 
    ��  ��:
    ����˵��������������
*/
void test_within_noise(void)
{

	  ADS1255_RESET();
	  ADC_Parameter_Config();
    Delay_us(1000);//��ʱ���ȴ��ź�Դ�ȶ�
    //�ɼ�2048����
	  AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);
    
}
/*******************************************************************************
    ��������test_impulse_response
    ��  ��: 
    ��  ��:
    ����˵����������Ӧ���ԣ����׼����
*/
void test_impulse_response(void)
{
    
    signals_ON(Pulsewave12bit_2HZ_75);
    //�л��ź�����ͨ��Ϊ����ͨ��
	  #ifdef Stack_X
    SwitchToSTDSigTest_X;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Y
    SwitchToSTDSigTest_Y;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Z
    SwitchToSTDSigTest_Z;/* ģ�⿪���л��������ź� */
    #endif
	  ADC_Parameter_Config_Selftest_start();
    Delay_us(50);
    //�ɼ�2048����
	  AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);
    //�л��ź�����ͨ��Ϊ�������ź�
	  #ifdef Stack_X
    SwitchToSampleSig_X;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Y
    SwitchToSampleSig_Y;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Z
    SwitchToSampleSig_Z;/* ģ�⿪���л��������ź� */
    #endif
		ADC_Parameter_Config();
    //�رձ�׼�ź�Դ�Է�ֹ����
    signals_OFF();
    
}
/*******************************************************************************
    ��������test_conformance
    ��  ��: 
    ��  ��:
    ����˵����һ���Բ��ԣ����׼�����ź�
*/
void test_conformance(void)
{
    //����30HZ 1Vpp��׼�����ź�
    signals_ON(signal_sina_30HZ);
    //�л��ź�����ͨ��Ϊ����ͨ��
    #ifdef Stack_X
    SwitchToSTDSigTest_X;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Y
    SwitchToSTDSigTest_Y;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Z
    SwitchToSTDSigTest_Z;/* ģ�⿪���л��������ź� */
    #endif
	  ADC_Parameter_Config_Selftest_start();
    Delay_us(50);
    //�ɼ�2048����
	  AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);
    //�л��ź�����ͨ��Ϊ�������ź�
    #ifdef Stack_X
    SwitchToSampleSig_X;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Y
    SwitchToSampleSig_Y;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Z
    SwitchToSampleSig_Z;/* ģ�⿪���л��������ź� */
    #endif
		ADC_Parameter_Config();
    //�رձ�׼�ź�Դ�Է�ֹ����
    signals_OFF();
}
/*******************************************************************************
    ��������test_crosstalk
    ��  ��: 
    ��  ��:
    ����˵�����������ԣ������׼�����ź�
*/
void test_crosstalk(void)
{
    //����30HZ 1Vpp��׼�����ź�
    signals_ON(signal_sina_30HZ);
    //�л��ź�����ͨ��Ϊ����ͨ��
	  #ifdef Stack_X
    SwitchToSTDSigTest_X;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Y
    SwitchToSampleSig_Y;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Z
    SwitchToSampleSig_Y;/* ģ�⿪���л��������ź� */
    #endif
	  ADC_Parameter_Config_Selftest_start();
    Delay_us(50);
    //�ɼ�2048����
	  AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);
    //�л��ź�����ͨ��Ϊ�������ź�
    #ifdef Stack_X
    SwitchToSampleSig_X;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Y
    SwitchToSampleSig_Y;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Z
    SwitchToSampleSig_Z;/* ģ�⿪���л��������ź� */
    #endif
		ADC_Parameter_Config();
    //�رձ�׼�ź�Դ�Է�ֹ����
    signals_OFF();
    
    
}
/*******************************************************************************
    ��������test_common_mode
    ��  ��: 
    ��  ��:
    ����˵������ģ���ԣ������׼�����ź�
*/
void test_common_mode(void)
{//�ݲ�֧��
    //����30HZ 1Vpp��׼�����ź�
    signals_ON(signal_sina_7_8125HZ_776);
    //�л��ź�����ͨ��Ϊ����ͨ��
    #ifdef Stack_X
    SwitchToSTDSigTest_X;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Y
    SwitchToSTDSigTest_Y;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Z
    SwitchToSTDSigTest_Z;/* ģ�⿪���л��������ź� */
    #endif
	  ADC_Parameter_Config_Selftest_start();
    Delay_us(50);
    
    //�ɼ�2048����
	  AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);
    //�л��ź�����ͨ��Ϊ�������ź�
    #ifdef Stack_X
    SwitchToSampleSig_X;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Y
    SwitchToSampleSig_Y;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Z
    SwitchToSampleSig_Z;/* ģ�⿪���л��������ź� */
    #endif
		ADC_Parameter_Config();
    //�رձ�׼�ź�Դ�Է�ֹ����
    signals_OFF();    
}
/*******************************************************************************
    ��������test_distortion
    ��  ��: 
    ��  ��:
    ����˵����������ԣ������׼�����ź�
*/
void test_distortion(void)
{//�ݲ�֧��
    //����30HZ 1Vpp��׼�����ź�
    signals_ON(signal_sina_7_8125HZ_97);
    //�л��ź�����ͨ��Ϊ����ͨ��
    #ifdef Stack_X
    SwitchToSTDSigTest_X;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Y
    SwitchToSTDSigTest_Y;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Z
    SwitchToSTDSigTest_Z;/* ģ�⿪���л��������ź� */
    #endif
	  ADC_Parameter_Config_Selftest_start();
    Delay_us(50);
    //�ɼ�2048����
	  AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);
    //�л��ź�����ͨ��Ϊ�������ź�
		#ifdef Stack_X
    SwitchToSampleSig_X;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Y
    SwitchToSampleSig_Y;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Z
    SwitchToSampleSig_Z;/* ģ�⿪���л��������ź� */
    #endif
		ADC_Parameter_Config();
    //�رձ�׼�ź�Դ�Է�ֹ����
    signals_OFF();
}

/*******************************************************************************
    ��������test_distortion
    ��  ��: 
    ��  ��:
    ����˵����������� 
		2017-09-12
*/
void test_res(void)
{//�ݲ�֧��
    //����7.8125HZ 97%��׼�����ź�
    signals_ON(signal_sina_7_8125HZ_776);
    //�л��ź�����ͨ��Ϊ����ͨ��
    #ifdef Stack_X
    SwitchToSTDSigTest_X;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Y
    SwitchToSTDSigTest_Y;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Z
    SwitchToSTDSigTest_Z;/* ģ�⿪���л��������ź� */
    #endif
	  ADC_Parameter_Config_Selftest_start();
    Delay_us(50);
    //�ɼ�2048����
	  AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);
    //�л��ź�����ͨ��Ϊ�������ź�
		#ifdef Stack_X
    SwitchToSampleSig_X;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Y
    SwitchToSampleSig_Y;/* ģ�⿪���л��������ź� */
    #endif
		#ifdef Stack_Z
    SwitchToSampleSig_Z;/* ģ�⿪���л��������ź� */
    #endif
		ADC_Parameter_Config();
    //�رձ�׼�ź�Դ�Է�ֹ����
    signals_OFF();
}

//void auto_test(void)
//{
//    //����DA���
//    int i=0;
//    DAC_Cmd(DAC_Channel_1, ENABLE);
//    while(1)
//    {
//        DAC_SetChannel1Data(DAC_Align_12b_R,i);		/* 12λ�Ҷ��룬���ǲ�ƫ��Ϊ0 */
//        //StartTimer(DAC_TIMER,2);
//        //while(CheckTimer(DAC_TIMER)!= 1)
//        Delay_us(100);//����Լ10ms����ֵ0-1024��Ӧ0-1v
//        i++;
//        if(i>1023)
//            i=0;
//    }
//}

/*******************************************************************************************************
���������Բ���
����һDA��������źţ�AD�Դ���������źź�DA�źŵ��Ӳɼ�
********************************************************************************************************/


/*******************************************************************************
    ��������sensor_test_within_noise
    ��  ��: 
    ��  ��:
    ����˵�����������������ԣ�ֱ�Ӳɼ���������̬ʱ������
		2017-09-12
*/
void sensor_test_within_noise(void)
{
	  Delay_us(50);
		AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);
}


/*******************************************************************************
    ��������sensor_test_common_mode
    ��  ��: 
    ��  ��:
    ����˵������������ģ���� �������ź�
		2017-09-12
*/
void sensor_test_common_mode(void)
{
	//�򿪲���ͨ�����أ����
	SENSOR_CHECK_ON;
	//����31.25hz 77.6%��������ź�
	signals_ON(signal_sina_30HZ);
	#ifdef Stack_X
	SwitchToSTDSigTest_X;/* ģ�⿪���л��������ź� */
	#endif
	#ifdef Stack_Y
	SwitchToSTDSigTest_Y;/* ģ�⿪���л��������ź� */
	#endif
	#ifdef Stack_Z
	SwitchToSTDSigTest_Z;/* ģ�⿪���л��������ź� */
	#endif
	ADC_Parameter_Config_Selftest_start();
	Delay_us(50);
	AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);
	//�رձ�׼�ź�Դ�Է�ֹ����
  signals_OFF();
	//�رղ���ͨ��
	SENSOR_CHECK_OFF;
	 //�л��ź�����ͨ��Ϊ�������ź�
	#ifdef Stack_X
	SwitchToSampleSig_X;/* ģ�⿪���л�������������ź� */
	#endif
	#ifdef Stack_Y
	SwitchToSampleSig_Y;/* ģ�⿪���л�������������ź� */
	#endif
	#ifdef Stack_Z
	SwitchToSampleSig_Z;/* ģ�⿪���л�������������ź� */
	#endif
	ADC_Parameter_Config();
}
/*******************************************************************************
    ��������sensor_test_impulse_response
    ��  ��: 
    ��  ��:
    ����˵����������������Ӧ���ԣ����������ź�
		2017-09-13
*/
void sensor_test_impulse_response(void)
{
	//�򿪲���ͨ�����أ����
	SENSOR_CHECK_ON;
	signals_ON(Pulsewave12bit_2HZ_75);
	#ifdef Stack_X
	SwitchToSTDSigTest_X;/* ģ�⿪���л��������ź� */
	#endif
	#ifdef Stack_Y
	SwitchToSTDSigTest_Y;/* ģ�⿪���л��������ź� */
	#endif
	#ifdef Stack_Z
	SwitchToSTDSigTest_Z;/* ģ�⿪���л��������ź� */
	#endif
	ADC_Parameter_Config_Selftest_start();
	Delay_us(50);
	AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);
	//�رձ�׼�ź�Դ�Է�ֹ����
  signals_OFF();
	//�رղ���ͨ��
	SENSOR_CHECK_OFF;
	//�л��ź�����ͨ��Ϊ�������ź�
	#ifdef Stack_X
	SwitchToSampleSig_X;/* ģ�⿪���л�������������ź� */
	#endif
	#ifdef Stack_Y
	SwitchToSampleSig_Y;/* ģ�⿪���л�������������ź� */
	#endif
	#ifdef Stack_Z
	SwitchToSampleSig_Z;/* ģ�⿪���л�������������ź� */
	#endif
	ADC_Parameter_Config();
	
}

/*******************************************************************************
    ��������sensor_test_distortion
    ��  ��: 
    ��  ��:
    ����˵�����������������
		2017-09-13
*/
void sensor_test_distortion(void)
{
	//�򿪲���ͨ�����أ����
	SENSOR_CHECK_ON;
	signals_ON(signal_sina_7_8125HZ_97);
	#ifdef Stack_X
	SwitchToSTDSigTest_X;/* ģ�⿪���л��������ź� */
	#endif
	#ifdef Stack_Y
	SwitchToSTDSigTest_Y;/* ģ�⿪���л��������ź� */
	#endif
	#ifdef Stack_Z
	SwitchToSTDSigTest_Z;/* ģ�⿪���л��������ź� */
	#endif
	ADC_Parameter_Config_Selftest_start();
	Delay_us(50);
	AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);
	//�رձ�׼�ź�Դ�Է�ֹ����
  signals_OFF();
	//�رղ���ͨ��
	SENSOR_CHECK_OFF;
	//�л��ź�����ͨ��Ϊ�������ź�
	#ifdef Stack_X
	SwitchToSampleSig_X;/* ģ�⿪���л�������������ź� */
	#endif
	#ifdef Stack_Y
	SwitchToSampleSig_Y;/* ģ�⿪���л�������������ź� */
	#endif
	#ifdef Stack_Z
	SwitchToSampleSig_Z;/* ģ�⿪���л�������������ź� */
	#endif
	ADC_Parameter_Config();
}

/*******************************************************************************
    ��������sensor_test_titl
    ��  ��: 
    ��  ��:
    ����˵������������б����
		2017-09-13
*/

void sensor_test_tilt(void)
{
	Delay_us(50);
	AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);
}

/*******************************************************************************
    ��������sensor_test_distortion
    ��  ��: 
    ��  ��:
    ����˵������������������
		2017-09-13
*/

void sensor_test_gravity(void)
{

	Delay_us(50);
	AD_RD_Data_Continous(&sample_data[0],&sample_data[2048*3],&sample_data[2048*3*2],2048);
}

//void auto_test(void)
//{
//    //����DA���
//    int i=0;
//    DAC_Cmd(DAC_Channel_1, ENABLE);
//    while(1)
//    {
//        DAC_SetChannel1Data(DAC_Align_12b_R,i);		/* 12λ�Ҷ��룬���ǲ�ƫ��Ϊ0 */
//        //StartTimer(DAC_TIMER,2);
//        //while(CheckTimer(DAC_TIMER)!= 1)
//        Delay_us(100);//����Լ10ms����ֵ0-1024��Ӧ0-1v
//        i++;
//        if(i>1023)
//            i=0;
//    }
//}

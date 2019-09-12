/*
  
* All rights reserved.
*
* File Name          : sensor_test.hc
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
#ifndef __SENSOR_TEST__
#define __SENSOR_TEST__

#include "ads1255.h"
//#include "math.h"
//#include <stdlib.h>

#define TestSigOutput		GPIO_Pin_4		/* PA4，标准测试信号DA输出端口 */

#define SwitchToSTDSigTest_X	ADS1255SetMux_sigle(Sensor_X,ADS1256_CHANNEL_SINGLE_5P);	/* 模拟开关切换至测试信号 */
#define SwitchToSampleSig_X	ADS1255SetMux_sigle(Sensor_X,ADS1256_CHANNEL_DIFFERENTIAL_0P1N);	/* 模拟开关切换至传感器输出信号 */
#define SwitchToSTDSigTest_Y	ADS1255SetMux_sigle(Sensor_Y,ADS1256_CHANNEL_SINGLE_5P);	/* 模拟开关切换至测试信号 */
#define SwitchToSampleSig_Y	ADS1255SetMux_sigle(Sensor_Y,ADS1256_CHANNEL_DIFFERENTIAL_0P1N)	/* 模拟开关切换至传感器输出信号 */
#define SwitchToSTDSigTest_Z	ADS1255SetMux_sigle(Sensor_Z,ADS1256_CHANNEL_SINGLE_5P);	/* 模拟开关切换至测试信号 */
#define SwitchToSampleSig_Z	ADS1255SetMux_sigle(Sensor_Z,ADS1256_CHANNEL_DIFFERENTIAL_0P1N);	/* 模拟开关切换至传感器输出信号 */

#define SwitchToSTDSigTest_X_1	ADS1255SetMux_sigle(Sensor_X,ADS1256_CHANNEL_SINGLE_0P);	/* 模拟开关切换至测试信号 */

//外部调用接口
void sensor_test_init(void);
void momal_sample_gpio_init(void);
void test_gain_Pulsewave(void);
void test_gain(void);
void test_within_noise(void);
void test_impulse_response(void);
void test_conformance(void);
void test_crosstalk(void);
void test_common_mode(void);
void test_distortion(void);
void test_res(void);

void background_noise(void);
void sensor_test_within_noise(void);
void sensor_test_common_mode(void);
void sensor_test_impulse_response(void);
void sensor_test_distortion(void);
void sensor_test_tilt(void);
void sensor_test_gravity(void);




//void auto_test(void);
#endif 

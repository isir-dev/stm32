/*
* Copyright (c) 2012,西南科技大学
* All rights reserved.
*
* File Name          : seismic_socket.h
* Description        : 
*
* Version            : V1.0
* Author             : liu Yong
* Date               : 10/28/2015
*
* OriginalVersion    : V1.0
* Original Author    : liu Yong
* Date               : 10/18/2015
*/
#ifndef __UART_H
#define __UART_H

#include "stm32f4xx.h"

#define MaxNodeNum 5
//#define LOCAL_ADDRESS 0x01
#define BRODCAST_ADDRESS 0xFF
#define HOST_ADDRESS 0x00

extern int Local_node_num;//定义节点号

//#define SAMPLE_BUF_LENGTH 100
//#define DIRECTION_BUF_LENGTH 6
//#define NOISE_BUF_LENGTH 108

#define ONLINE_REQUEST 0x01
#define ONLINE_REPLY 0x02
#define PARAMETER_CONFIG_REQUEST 0x03
#define PARAMETER_CONFIG_REPLY 0x04
#define STRETCH_REQUEST 0x05
#define STRETCH_REPLY 0x06
#define APPROACH_REQUEST 0x07
#define APPROACH_REPLY 0x08
#define LOAD_DIRECTION_REQUEST 0x09
#define LOAD_DIRECTION_REPLY 0x0A

#define SYNC_REQUEST 0x0B
#define SYNC_REPLY 0x0C
#define PRE_REQUEST 0x0D
#define PRE_REPLY 0x0E

#define SAMPLE_FINISHED_REQUEST 0x0F
#define SAMPLE_FINISHED_REPLY 0x10
#define LOAD_DATA_REQUEST 0x11
#define LOAD_DATA_REPLY 0x12
#define LOAD_NOISE_REQUEST 0x13
#define LOAD_NOISE_REPLY 0x14

#define MOT_STATE_REQUEST 0x15
#define MOT_STATE_REPLY 0x16
#define MOT_OFF_REQUEST 0x17
#define MOT_OFF_REPLY 0x18

//自检模块
#define TEST_GAIN_REQUEST 0x19
#define TEST_GAIN_REPLY 0x1A
#define TEST_WITHIN_NOISE_REQUEST 0x1B
#define TEST_WITHIN_NOISE_REPLY 0x1C
#define TEST_IMPULSE_RESPONSE_REQUEST 0x1D
#define TEST_IMPULSE_RESPONSE_REPLY 0x1E
#define TEST_CONFORMANCE_REQUEST 0x1F
#define TEST_CONFORMANCE_REPLY 0x20
#define TSET_CROSSTALK_REQUEST 0x21
#define TEST_CROSSTALK_REPLY 0x22
#define TEST_COMMON_MODE_REQUEST 0x23
#define TEST_COMMON_MODE_REPLY 0x24
#define TEST_DISTORTION_REQUEST 0x25
#define TEST_DISTORTION_REPLY 0x26

#define TEST_RESISTANCE_REQUEST 0x27
#define TEST_RESISTANCE_REPLY 0x28

//添加传感器检测模块
#define TEST_WITHIN_NOISE_SENSOR_REQUEST 0x29
#define TEST_WITHIN_NOISE_SENSOR_REPLY 0x2A
#define TEST_DISTORTION_SENSOR_REQUEST 0x2B
#define TEST_DISTORTION_SENSOR_REPLY 0x2C
#define TEST_COMMON_MODE_SENSOR_REQUEST 0x2D
#define TEST_COMMON_MODE_SENSOR_REPLY 0x2E
#define TEST_IMPULSE_RESPONSE_SENSOR_REQUEST 0x2F 
#define TEST_IMPULSE_RESPONSE_SENSOR_REPLY 0x30
#define TEST_TILT_SENSOR_REQUEST 0x31
#define TEST_TILT_SENSOR_REPLY 0x32
#define TEST_GRAVITY_SENSOR_REQUEST 0x33 
#define TEST_GRAVITY_SENSOR_REPLY 0x34
#define MOT_STATE_REPLY_WORKING 0x36
#define MOT_STATE_REPLY_UNWORKING 0x37

#define CONFIG_ADDRESS_REQUEST 0x38
#define CONFIG_ADDRESS_REPLY 0x39

//添加电子罗盘矫正模块
#define READADDR_REQUEST 0x40
#define READADDR_REQUEST_REPLY 0x41

//#define ACC_CALIBRATION_REQUEST 0x42
//#define ACC_CALIBRATION_REPLY   0x43

//#define MAG_CALIBRATION_REQUEST 0x44
//#define MAG_CALIBRATION_REPLY   0x45 

//#define UPDATE_ACC_DATA_REQUEST 0x46
//#define UPDATE_ACC_DATA_REPLY   0x47 

//#define UPDATE_MAG_DATA_REQUEST 0x48 
//#define UPDATE_MAG_DATA_REPLY   0x49 

//#define SAVE_CONFIFG_REQUEST    0x4a
//#define SAVE_CONFIFG_REPLY      0x4b

#define RESET_DIRECTION_CONFIG_REQUEST 0x4c
#define RESET_DIRECTION_CONFIG_REPLY   0x4d

#define SET_DIRECTION_UP_REQUEST 0x4e  //垂直安装
#define SET_DIRECTION_UP_REPLY 0x4f //

#define SET_DIRECTION_HOR_REQUEST 0x50  //水平安装
#define SET_DIRECTION_HOR_REPLY 0x51 

#define SET_ALGORITHM_REQUEST 0x52
#define SET_ALGORITHM_REPLY 0x53

#define SET_GYRO_RANGE_REQUEST 0x54
#define SET_GYRO_RANGE_REPLY 0x55

#define SET_GYRO_BANDWIDTH_REQUEST 0x56
#define SET_GYRO_BANDWIDTH_REPLY 0x57

#define SET_GYRO_ACCELERATION_RANGE_REQUEST 0x58
#define SET_GYRO_ACCELERATION_RANGE_REPLY 0x59

#define ACC_START_CALIBRATION_REQUEST 0x5a 
#define ACC_START_CALIBRATION_REPLY 0x5b
 
#define ACC_UP_DATA_REQUEST  0x5c
#define ACC_UP_DATA_REPLY		0x5d

#define ACC_WRITE_PARAMETER_REQUEST  0x5e
#define ACC_WRITE_PARAMETER_REPLY		0x5f

#define MAG_START_CALIBRATION_REQUEST  0x60
#define MAG_START_CALIBRATION_REPLY		0x61

#define MAG_UP_DATA_REQUEST  0x62
#define MAG_UP_DATA_REPLY		0x63


#define MAG_WRITE_PARAMETER_REQUEST  0x64
#define MAG_WRITE_PARAMETER_REQUEST_REPLY		0x65

#define GYRO_UP_DATAS_REQUEST 0X66
#define GYRO_UP_DATAS_REPLY 0X67
//待添加
#define SET_ALGORITHM_6_REQUEST 0x68

#define SET_GYRO_ACCELERATION_4_RANGE_REQUEST 0x69
#define SET_GYRO_ACCELERATION_8_RANGE_REQUEST 0x6a
#define SET_GYRO_ACCELERATION_16_RANGE_REQUEST 0x6b

#define SET_GYRO_500_RANGE_REQUEST 0x6c
#define SET_GYRO_1000_RANGE_REQUEST 0x6d
#define SET_GYRO_2000_RANGE_REQUEST 0x6e

#define SET_GYRO_256_BANDWIDTH_REQUEST 0x6f
#define SET_GYRO_188_BANDWIDTH_REQUEST 0x70
#define SET_GYRO_98_BANDWIDTH_REQUEST  0x71
#define SET_GYRO_42_BANDWIDTH_REQUEST  0x72
#define SET_GYRO_10_BANDWIDTH_REQUEST  0x73
#define SET_GYRO_5_BANDWIDTH_REQUEST   0x74



void uart_init(void);
int Data_type_conversion_uint8_int(uint8_t x);
uint8_t Data_type_conversion_int_uint8(int x);
void Local_NodeNum_Set_RFlash(void);
#endif /* __UART_H */


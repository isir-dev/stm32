/*
  
* All rights reserved.
*
* File Name          : CRC_8.h
* Description        :  usnp应用层辅助实现
*
* Version            : V1.1
* Author             : liu Yong
* Date               : 11/20/2012
*
* OriginalVersion    : V1.0
* Original Author    : oylf1986
* Date               : 11/19/2012
* Reference : VSP（使用中文文字输入后下一行再输入代码时需要用英文标点符号结束，不然有红色波浪线）;
*            使用颠倒的直驱表法,其它被注释方法为驱动表法和直驱查表法，还包括一种从文件中读出数据进行校验;
*/
//外部调用接口
unsigned char CRC8_Tab(unsigned char *ucPtr,   unsigned char ucLen);
unsigned char FindCRC(unsigned char *data,unsigned char datalen);//直接计算法计算CRC8;



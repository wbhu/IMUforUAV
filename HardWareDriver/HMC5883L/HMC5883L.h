/******************** (C) COPYRIGHT 2015 DUT ********************************
 * 作者    ：胡文博
 * 文件名  ：HMC5883L.h
 * 描述    ：HMC5883L驱动头文件
 * 日期    ：2015/11/30 12:43:38
 * 联系方式：1461318172（qq）
**********************************************************************************/

#ifndef __HMC5883L_H
#define __HMC5883L_H

#include "stm32f4xx.h"
#include "IOI2C.h"
#include "delay.h"

#define HMC58X3_ADDR 0x3C // 7 bit address of the HMC58X3 used with the Wire library
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2
// HMC58X3 register map. For details see HMC58X3 datasheet
#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_R_XM 3
#define HMC58X3_R_XL 4

#define HMC58X3_R_YM (7)  //!< Register address for YM.
#define HMC58X3_R_YL (8)  //!< Register address for YL.
#define HMC58X3_R_ZM (5)  //!< Register address for ZM.
#define HMC58X3_R_ZL (6)  //!< Register address for ZL.

#define HMC58X3_R_STATUS 9
#define HMC58X3_R_IDA 10
#define HMC58X3_R_IDB 11
#define HMC58X3_R_IDC 12

extern unsigned char HMC5883_calib;	//正在标定中？
extern int16_t  HMC5883_maxx, HMC5883_maxy, HMC5883_maxz,
       HMC5883_minx, HMC5883_miny, HMC5883_minz;

void HMC5883L_SetUp(void);	//初始化
void HMC58X3_getID(char id[3]);	//读芯片ID
void HMC58X3_getValues(int16_t *x, int16_t *y, int16_t *z); //读ADC
void HMC58X3_mgetValues(float *arry); //IMU 专用的读取磁力计值
void HMC58X3_getlastValues(int16_t *x, int16_t *y, int16_t *z);
void HMC5883L_Save_Calib(void);	  //保存磁力计标定
void HMC5883L_Start_Calib(void);  //开始磁力计标定

#endif

//------------------End of File----------------------------

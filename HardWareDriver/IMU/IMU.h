/******************** (C) COPYRIGHT 2015 DUT ********************************
 * 作者    ：胡文博
 * 文件名  ：IMU.h
 * 描述    ：姿态解算算法头文件
 * 日期    ：2015/11/30 12:43:38
 * 联系方式：1461318172（qq）
**********************************************************************************/
/********************
  * @attention
  *
  *占用STM32 资源：
  *1. 使用Tim7定时器 产生us级的系统时间
  ******************************************************************************
 */

#ifndef __IMU_H
#define __IMU_H

#include "common.h"

#include <math.h>
#define M_PI  (float)3.1415926535
#define micros() TIM5->CNT

//解算后的姿态输出，供外部程序调用
extern volatile float IMU_Pitch, IMU_Roll, IMU_Yaw, ACC_Pitch, ACC_Roll;	 //单位 度
//陀螺仪的输出，
extern volatile float IMU_GYROx, IMU_GYROy, IMU_GYROz;	//单位  度每秒
extern volatile float acc_vector;  //当前加速度感应到的力合  M/S^2。
//Mini IMU AHRS 解算的API
void IMU_init(void); //初始化
void IMU_getYawPitchRoll(float *ypr);  //更新姿态  需要定期调用
//uint32_t micros(void);	//读取系统上电后的时间  单位 us
void Initialize_Q(void);

#endif

//------------------End of File----------------------------

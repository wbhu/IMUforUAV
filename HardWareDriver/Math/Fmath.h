/******************** (C) COPYRIGHT 2015 DUT ********************************
 * 作者    ：胡文博
 * 文件名  ：Fmath.c
 * 描述    ：自定义数学库头文件
 * 日期    ：2015/11/30 12:43:38
 * 联系方式：1461318172（qq）
**********************************************************************************/



#ifndef __FMATH_H
#define __FMATH_H


#define PitchRollEXP  50   //0.5
#define PitchRollRate 100  //1.0

#define ThrMid   0
#define Thr_EXP  40

#include "common.h"

float Math_fConstrain(float value, float min, float max);
int16_t Math_Constrain(int16_t value, int16_t min, int16_t max);
int16_t Math_abs(int16_t value);
int16_t Math_min(int16_t value1, int16_t value2);
int16_t Math_max(int16_t value1, int16_t value2);
void Math_init_EXP(void);
int16_t Math_ThrEXP(int16_t RCThr);
int16_t Math_AngelEXP(int16_t in);

#endif

//------------------End of File----------------------------

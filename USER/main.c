/******************** (C) COPYRIGHT 2015 DUT ********************************
 * 作者    ：胡文博
 * 文件名  ：main.c
 * 描述    ：主函数
 * 日期    ：2015/11/30 12:43:38
 * 联系方式：1461318172（qq）
**********************************************************************************/

#include "common.h"

#define reportHz (500)
#define uploadTime 1000000/reportHz

// #define REPORT() 	{	\
//				printf("test ok\n");	\
//				printf("ahrs:%.2f,%.2f,%.2f,%.2f,%.2f\n",IMU_Yaw,IMU_Pitch,IMU_Roll,ACC_Pitch,ACC_Roll);	\
//				printf("ypr:%.2f,%.2f,%.2f\n",ypr[0],ypr[1],ypr[2]);	\
//				printf("GYRO:%.2f,%.2f,%.2f\n",IMU_GYROx, IMU_GYROy, IMU_GYROz);	\
//				printf("acc_vector:%.2f\n",acc_vector);		\
//			}
//

#define REPORT()	{		\
				printf("ypr:%.2f,%.2f,%.2f\n", ypr[0], ypr[1], ypr[2]);	\
			}

//初始化TIM5 32位定时器，用于做系统的时钟。
void Initial_System_Timer(void)
{
	RCC->APB1ENR |= 0x0008;	//使能TIM5时钟
	TIM5->CR1 = 0x0080; //TIMx_ARR buffered  upcounter
	TIM5->CR2 = 0x0000;
	TIM5->CNT = 0x0000;
	TIM5->ARR = 0xFFFFFFFF;
	TIM5->PSC = 84 - 1;	//分出 1M 的时钟 保证每个周期为1us
	TIM5->EGR = 0x0001;
	TIM5->CR1 |= 0x0001; //启动定时器
}







int main(void)
{
	u32 now = 0, lastTime = 0;
	
	float ypr[3];
	int16_t AX = 3, AY = 3, AZ = 3, GX = 3, GY = 3, GZ = 3;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); //中断优先级分组
	delay_init(168);		//延时初始化  并启动开机时间。


	uart_init(115200);

	delay_ms(500);
	IIC_Init();
	delay_ms(50);
	Initial_System_Timer();  //启动系统主定时器
	IMU_init();
	Initialize_Q();	//初始化四元数


	now = lastTime = micros();
	while(1)
	{
		IMU_getYawPitchRoll( ypr);	//姿态更新task
		now = micros();
		if(now < lastTime)
		{
			if((0xffffffff - lastTime + now) > uploadTime)
			{
				REPORT();
				lastTime = now;
			}
		}
		else
		{
			if((now - lastTime) > uploadTime)
			{
				REPORT();
				lastTime = now;
			}
		}
	}
}




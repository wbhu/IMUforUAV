/******************** (C) COPYRIGHT 2015 DUT ********************************
 * 作者    ：胡文博
 * 文件名  ：MPU6050.c
 * 描述    ：mpu6050驱动
 * 日期    ：2015/11/30 12:43:38
 * 联系方式：1461318172（qq）
**********************************************************************************/
/********************
  * @attention
  *
  *占用资源：
  *1. I2C 接口访问MPU6050
  *2. 读取 PC9 引脚状态 以确定MPU6050 完成了新的转换

  ******************************************************************************
 */

#include "MPU6050.h"
#include "common.h"
#include "Fmath.h"

#define  Buf_Size 10

uint8_t buffer[14], Buf_index = 0;	 //数据读取缓冲区

int16_t  MPU6050_FIFO[6][11];
int16_t Gx_offset = 0, Gy_offset = 0, Gz_offset = 0;
int16_t Gyro_ADC[3], ACC_ADC[3];


//读取队列 的平均值
int16_t MPU6050_getAvg(int16_t *buff, int size)
{
	int32_t sum = 0;
	int i;
	for(i = 0; i < size; i++)
	{
		sum += buff[i];
	}
	return sum / size;
}

/**************************实现函数********************************************
*函数原型:		void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
*功　　能:	    将新的ADC数据更新到 FIFO数组，进行滤波处理
*******************************************************************************/
void  MPU6050_newValues(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz)
{
	int16_t new;

	//Gyro_ADC[0] 数组将会参与PID的运算
	new = (Gyro_ADC[0] * 3 + (gx - Gx_offset)) / 4; // range: +/- 8192; +/- 2000 deg/sec
	Gyro_ADC[0] = Math_Constrain(new, Gyro_ADC[0] - 800, Gyro_ADC[0] + 800); //范围限定
	new = (Gyro_ADC[1] * 3 + (gy - Gy_offset)) / 4;
	Gyro_ADC[1] = Math_Constrain(new, Gyro_ADC[1] - 800, Gyro_ADC[1] + 800);
	new = (Gyro_ADC[2] * 3 + (gz - Gz_offset)) / 4;
	Gyro_ADC[2] = Math_Constrain(new, Gyro_ADC[2] - 800, Gyro_ADC[2] + 800);

	ACC_ADC[0]  = ax;
	ACC_ADC[1]  = ay;
	ACC_ADC[2]  = az;

	MPU6050_FIFO[0][Buf_index] = ax;
	MPU6050_FIFO[1][Buf_index] = ay;
	MPU6050_FIFO[2][Buf_index] = az;
	MPU6050_FIFO[3][Buf_index] = gx;
	MPU6050_FIFO[4][Buf_index] = gy;
	MPU6050_FIFO[5][Buf_index] = gz;
	Buf_index = (Buf_index + 1) % Buf_Size;//循环按列更新

	MPU6050_FIFO[0][10] = MPU6050_getAvg(MPU6050_FIFO[0], Buf_Size);
	MPU6050_FIFO[1][10] = MPU6050_getAvg(MPU6050_FIFO[1], Buf_Size);
	MPU6050_FIFO[2][10] = MPU6050_getAvg(MPU6050_FIFO[2], Buf_Size);
	MPU6050_FIFO[3][10] = MPU6050_getAvg(MPU6050_FIFO[3], Buf_Size);
	MPU6050_FIFO[4][10] = MPU6050_getAvg(MPU6050_FIFO[4], Buf_Size);
	MPU6050_FIFO[5][10] = MPU6050_getAvg(MPU6050_FIFO[5], Buf_Size);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setClockSource(uint8_t source)
*功　　能:	    设置  MPU6050 的时钟源
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
void MPU6050_setClockSource(uint8_t source)
{
	IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_setFullScaleGyroRange(uint8_t range)
{
	IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setFullScaleAccelRange(uint8_t range)
*功　　能:	    设置  MPU6050 加速度计的最大量程
*******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range)
{
	IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setSleepEnabled(uint8_t enabled)
*功　　能:	    设置  MPU6050 是否进入睡眠模式
				enabled =1   睡觉
			    enabled =0   工作
*******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled)
{
	IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		uint8_t MPU6050_getDeviceID(void)
*功　　能:	    读取  MPU6050 WHO_AM_I 标识	 将返回 0x68
*******************************************************************************/
uint8_t MPU6050_getDeviceID(void)
{

	IICreadBytes(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
	return buffer[0];
}

/**************************实现函数********************************************
*函数原型:		uint8_t MPU6050_testConnection(void)
*功　　能:	    检测MPU6050 是否已经连接
*******************************************************************************/
uint8_t MPU6050_testConnection(void)
{
	if(MPU6050_getDeviceID() == 0x68)  //0b01101000;
		return 1;
	else return 0;
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
{
	IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled)
{
	IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_initialize(void)
*功　　能:	    初始化 	MPU6050 以进入可用状态。
*******************************************************************************/
void MPU6050_initialize(void)
{
	int16_t temp[6];
	unsigned char i;

	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	/*
	PC9 为 输入且使能上拉电阻

	PC9  MPU6050 中断引脚
	*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	//应用配置　到GPIOC　
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO); //设置时钟
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//陀螺仪最大量程 +-2000度每秒
	MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);	//加速度度最大量程 +-2G
	MPU6050_setSleepEnabled(0); //进入工作状态
	MPU6050_setI2CMasterModeEnabled(0);	 //不让MPU6050 控制AUXI2C
	MPU6050_setI2CBypassEnabled(1);	 //主控制器的I2C与	MPU6050的AUXI2C	直通。控制器可以直接访问HMC5883L


	//配置MPU6050 的中断模式 和中断电平模式
	IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, 0);
	IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, 0);
	IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, 1);
	IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, 1);
	//开数据转换完成中断
	IICwriteBit(devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, 1);
	//增加MPU6050的低通滤波，大大减少了机架震动对姿态解算的影响
	IICwriteBits(devAddr, MPU6050_RA_CONFIG, 7, 8, MPU6050_DLPF); //设置MPU6050低通滤波

	for(i = 0; i < 10; i++) //更新FIFO数组
	{
		delay_us(50);
		MPU6050_getMotion6(&temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5]);
	}

}

/**************************实现函数********************************************
*函数原型:		unsigned char MPU6050_is_DRY(void)
*功　　能:	    检查 MPU6050的中断引脚，测试是否完成转换
返回 1  转换完成
0 数据寄存器还没有更新
*******************************************************************************/
unsigned char MPU6050_is_DRY(void)
{
	if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9) == Bit_SET)
	{
		return 1;
	}
	else return 0;
}

int16_t MPU6050_Lastax, MPU6050_Lastay, MPU6050_Lastaz  //上次值
, MPU6050_Lastgx, MPU6050_Lastgy, MPU6050_Lastgz;
/**************************实现函数********************************************
*函数原型:		void MPU6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
*功　　能:	    读取 MPU6050的当前测量值
*******************************************************************************/
void MPU6050_getMotion6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{

	if(MPU6050_is_DRY())
	{
		IICreadBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
		MPU6050_Lastax = (((int16_t)buffer[0]) << 8) | buffer[1];
		MPU6050_Lastay = (((int16_t)buffer[2]) << 8) | buffer[3];
		MPU6050_Lastaz = (((int16_t)buffer[4]) << 8) | buffer[5];
		//跳过温度ADC
		MPU6050_Lastgx = (((int16_t)buffer[8]) << 8) | buffer[9];
		MPU6050_Lastgy = (((int16_t)buffer[10]) << 8) | buffer[11];
		MPU6050_Lastgz = (((int16_t)buffer[12]) << 8) | buffer[13];
		MPU6050_newValues(MPU6050_Lastax, MPU6050_Lastay, MPU6050_Lastaz
				  , MPU6050_Lastgx, MPU6050_Lastgy, MPU6050_Lastgz);
		*ax  = MPU6050_FIFO[0][10];
		*ay  = MPU6050_FIFO[1][10];
		*az = MPU6050_FIFO[2][10];
		*gx  = MPU6050_FIFO[3][10] - Gx_offset;
		*gy = MPU6050_FIFO[4][10] - Gy_offset;
		*gz = MPU6050_FIFO[5][10] - Gz_offset;

#ifdef DEBUG
		printf("enter\n");
#endif
	}
	else
	{
		*ax = MPU6050_FIFO[0][10];//=MPU6050_FIFO[0][10];
		*ay = MPU6050_FIFO[1][10];//=MPU6050_FIFO[1][10];
		*az = MPU6050_FIFO[2][10];//=MPU6050_FIFO[2][10];
		*gx = MPU6050_FIFO[3][10] - Gx_offset; //=MPU6050_FIFO[3][10];
		*gy = MPU6050_FIFO[4][10] - Gy_offset; //=MPU6050_FIFO[4][10];
		*gz = MPU6050_FIFO[5][10] - Gz_offset; //=MPU6050_FIFO[5][10];
	}
}

void MPU6050_getlastMotion6(int16_t *ax, int16_t *ay,
			    int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
	*ax  = MPU6050_FIFO[0][10];
	*ay  = MPU6050_FIFO[1][10];
	*az = MPU6050_FIFO[2][10];
	*gx  = MPU6050_FIFO[3][10] - Gx_offset;
	*gy = MPU6050_FIFO[4][10] - Gy_offset;
	*gz = MPU6050_FIFO[5][10] - Gz_offset;
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_InitGyro_Offset(void)
*功　　能:	    读取 MPU6050的陀螺仪偏置
此时模块应该被静止放置。以测试静止时的陀螺仪输出
*******************************************************************************/
void MPU6050_InitGyro_Offset(void)
{
	unsigned char i;
	int16_t temp[6];
	int32_t	tempgx = 0, tempgy = 0, tempgz = 0;
	int32_t	tempax = 0, tempay = 0, tempaz = 0;
	Gx_offset = 0;
	Gy_offset = 0;
	Gz_offset = 0;
	for(i = 0; i < 50; i++)
	{
		delay_us(100);
		MPU6050_getMotion6(&temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5]);
		//LED_Change();
	}
	for(i = 0; i < 100; i++)
	{
		delay_us(200);
		MPU6050_getMotion6(&temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5]);
		tempax += temp[0];
		tempay += temp[1];
		tempaz += temp[2];
		tempgx += temp[3];
		tempgy += temp[4];
		tempgz += temp[5];
		//LED_Change();
	}

	Gx_offset = tempgx / 100; //MPU6050_FIFO[3][10];
	Gy_offset = tempgy / 100; //MPU6050_FIFO[4][10];
	Gz_offset = tempgz / 100; //MPU6050_FIFO[5][10];
	//

}

//------------------End of File----------------------------

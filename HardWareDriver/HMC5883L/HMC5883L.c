/******************** (C) COPYRIGHT 2015 DUT ********************************
 * 作者    ：胡文博
 * 文件名  ：HMC5883L.c
 * 描述    ：HMC5883L驱动
 * 日期    ：2015/11/30 12:43:38
 * 联系方式：1461318172（qq）
**********************************************************************************/
#include "HMC5883L.h"
#include "common.h"

#define  HMC5883_Buf_Size 10

float HMC5883_lastx, HMC5883_lasty, HMC5883_lastz;

//磁力计标定值
int16_t HMC5883_Offset_X = 0,
	HMC5883_Offset_Y = 0,
	HMC5883_Offset_Z = 0;

float  HMC5883_Scale_X = 1.0f,
       HMC5883_Scale_Y = 1.0f,
       HMC5883_Scale_Z = 1.0f;
//当前磁场的最大值和最小值
int16_t  HMC5883_maxx = 0, HMC5883_maxy = 0, HMC5883_maxz = 0,
	 HMC5883_minx = -0, HMC5883_miny = -0, HMC5883_minz = -0;
unsigned char HMC5883_calib = 0; //初始化完成标志

int8_t  HMC5883_Buf_index = 0;
int16_t  HMC5883_FIFO[3][11]; //磁力计滤波
void HMC58X3_getRaw(int16_t *x, int16_t *y, int16_t *z);

/**************************实现函数********************************************
*函数原型:	   unsigned char HMC5883_IS_newdata(void)
*功　　能:	   读取DRDY 引脚，判断是否完成了一次转换
 Low for 250 μsec when data is placed in the data output registers.
输入参数：  无
输出参数：  如果完成转换，则输出1  否则输出 0
*******************************************************************************/
unsigned char HMC5883_IS_newdata(void)
{
	if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8) == Bit_SET)
	{
		return 1;
	}
	else return 0;
}

/**************************实现函数********************************************
*函数原型:	   void HMC58X3_FIFO_init(void)
*功　　能:	   连续读取100次数据，以初始化FIFO数组
输入参数：  无
输出参数：  无
*******************************************************************************/
void HMC58X3_FIFO_init(void)
{
	int16_t temp[3];
	unsigned char i;
	for(i = 0; i < 50; i++)
	{
		HMC58X3_getRaw(&temp[0], &temp[1], &temp[2]);
		delay_us(200);  //延时再读取数据

	}
}

/**************************实现函数********************************************
*函数原型:	   void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
*功　　能:	   更新一组数据到FIFO数组
输入参数：  磁力计三个轴对应的ADC值
输出参数：  无
*******************************************************************************/
void  HMC58X3_newValues(int16_t x, int16_t y, int16_t z)
{
	unsigned char i ;
	int32_t sum = 0;

	HMC5883_FIFO[0][HMC5883_Buf_index] = x;
	HMC5883_FIFO[1][HMC5883_Buf_index] = y;
	HMC5883_FIFO[2][HMC5883_Buf_index] = z;
	HMC5883_Buf_index = (HMC5883_Buf_index + 1) % HMC5883_Buf_Size;//循环

	sum = 0;
	for(i = 0; i < 10; i++) 	//取数组内的值进行求和再取平均
	{
		sum += HMC5883_FIFO[0][i];
	}
	HMC5883_FIFO[0][10] = sum / 10;	//将平均值更新

	sum = 0;
	for(i = 0; i < 10; i++)
	{
		sum += HMC5883_FIFO[1][i];
	}
	HMC5883_FIFO[1][10] = sum / 10;

	sum = 0;
	for(i = 0; i < 10; i++)
	{
		sum += HMC5883_FIFO[2][i];
	}
	HMC5883_FIFO[2][10] = sum / 10;

	if(HMC5883_calib) //校正有效的话 采集标定值
	{
		if(HMC5883_minx > HMC5883_FIFO[0][10])HMC5883_minx = (int16_t)HMC5883_FIFO[0][10];
		if(HMC5883_miny > HMC5883_FIFO[1][10])HMC5883_miny = (int16_t)HMC5883_FIFO[1][10];
		if(HMC5883_minz > HMC5883_FIFO[2][10])HMC5883_minz = (int16_t)HMC5883_FIFO[2][10];

		if(HMC5883_maxx < HMC5883_FIFO[0][10])HMC5883_maxx = (int16_t)HMC5883_FIFO[0][10];
		if(HMC5883_maxy < HMC5883_FIFO[1][10])HMC5883_maxy = (int16_t)HMC5883_FIFO[1][10];
		if(HMC5883_maxz < HMC5883_FIFO[2][10])HMC5883_maxz = (int16_t)HMC5883_FIFO[2][10];

	}

} //HMC58X3_newValues

/**************************实现函数********************************************
*函数原型:	   void HMC58X3_writeReg(unsigned char reg, unsigned char val)
*功　　能:	   写HMC5883L的寄存器
输入参数：    reg  寄存器地址
			  val   要写入的值
输出参数：  无
*******************************************************************************/
void HMC58X3_writeReg(unsigned char reg, unsigned char val)
{
	IICwriteByte(HMC58X3_ADDR, reg, val);
}

/**************************实现函数********************************************
*函数原型:	  void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z)
*功　　能:	   写HMC5883L的寄存器
输入参数：    reg  寄存器地址
			  val   要写入的值
输出参数：  无
*******************************************************************************/
void HMC58X3_getRaw(int16_t *x, int16_t *y, int16_t *z)
{
	unsigned char vbuff[6];
	vbuff[0] = vbuff[1] = vbuff[2] = vbuff[3] = vbuff[4] = vbuff[5] = 0;
	IICreadBytes(HMC58X3_ADDR, HMC58X3_R_XM, 6, vbuff);
	HMC58X3_newValues(((int16_t)vbuff[0] << 8) | vbuff[1], ((int16_t)vbuff[4] << 8) | vbuff[5], ((int16_t)vbuff[2] << 8) | vbuff[3]);
	*x = HMC5883_FIFO[0][10];
	*y = HMC5883_FIFO[1][10];
	*z = HMC5883_FIFO[2][10];
}

/**************************实现函数********************************************
*函数原型:	  void HMC58X3_getValues(int16_t *x,int16_t *y,int16_t *z)
*功　　能:	   读取 磁力计的当前ADC值
输入参数：    三个轴对应的输出指针
输出参数：  无
*******************************************************************************/
void HMC58X3_getlastValues(int16_t *x, int16_t *y, int16_t *z)
{
	*x = HMC5883_FIFO[0][10];
	*y = HMC5883_FIFO[1][10];
	*z = HMC5883_FIFO[2][10];
}

/**************************实现函数********************************************
*函数原型:	  void HMC58X3_mgetValues(float *arry)
*功　　能:	   读取 校正后的 磁力计ADC值
输入参数：    输出数组指针
输出参数：  无
*******************************************************************************/
void HMC58X3_mgetValues(float *arry)
{
	int16_t xr, yr, zr;
	HMC58X3_getRaw(&xr, &yr, &zr);
	arry[0] = HMC5883_lastx = (float)(((float)(xr - HMC5883_Offset_X)) * HMC5883_Scale_X);
	arry[1] = HMC5883_lasty = (float)(((float)(yr - HMC5883_Offset_Y)) * HMC5883_Scale_Y);
	arry[2] = HMC5883_lastz = (float)(((float)(zr - HMC5883_Offset_Z)) * HMC5883_Scale_Z);
}

/**************************实现函数********************************************
*函数原型:	  void HMC58X3_setGain(unsigned char gain)
*功　　能:	   设置 5883L的增益
输入参数：     目标增益 0-7
输出参数：  无
*******************************************************************************/
void HMC58X3_setGain(unsigned char gain)
{
	// 0-7, 1 default
	if (gain > 7) return;
	HMC58X3_writeReg(HMC58X3_R_CONFB, gain << 5);
}

/**************************实现函数********************************************
*函数原型:	  void HMC58X3_setMode(unsigned char mode)
*功　　能:	   设置 5883L的工作模式
输入参数：     模式
输出参数：  无
*******************************************************************************/
void HMC58X3_setMode(unsigned char mode)
{
	if (mode > 2)
	{
		return;
	}
	HMC58X3_writeReg(HMC58X3_R_MODE, mode);
	delay_us(100);
}

/**************************实现函数********************************************
*函数原型:	  void HMC58X3_init(u8 setmode)
*功　　能:	   设置 5883L的工作模式
输入参数：     模式
输出参数：  无
*******************************************************************************/
void HMC58X3_init(u8 setmode)
{

	/*
	PC8 为 输入且使能上拉电阻

	PC8  5883 中断引脚
	*/
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	//应用配置　到GPIOC　
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	if (setmode)
	{
		HMC58X3_setMode(0);
	}

	HMC58X3_writeReg(HMC58X3_R_CONFA, 0x70); // 8 samples averaged, 75Hz frequency, no artificial bias.
	HMC58X3_writeReg(HMC58X3_R_CONFB, 0xA0);
	HMC58X3_writeReg(HMC58X3_R_MODE, 0x00);

}

/**************************实现函数********************************************
*函数原型:	  void HMC58X3_setDOR(unsigned char DOR)
*功　　能:	   设置 5883L的 数据输出速率
输入参数：     速率值
0 -> 0.75Hz  |   1 -> 1.5Hz
2 -> 3Hz     |   3 -> 7.5Hz
4 -> 15Hz    |   5 -> 30Hz
6 -> 75Hz
输出参数：  无
*******************************************************************************/
void HMC58X3_setDOR(unsigned char DOR)
{
	if (DOR > 6) return;
	HMC58X3_writeReg(HMC58X3_R_CONFA, DOR << 2);
}

/**************************实现函数********************************************
*函数原型:	  void HMC58X3_getID(char id[3])
*功　　能:	   读取芯片的ID
输入参数：     	ID存放的数组
输出参数：  无
*******************************************************************************/
void HMC58X3_getID(char id[3])
{
	id[0] = I2C_ReadOneByte(HMC58X3_ADDR, HMC58X3_R_IDA);
	id[1] = I2C_ReadOneByte(HMC58X3_ADDR, HMC58X3_R_IDB);
	id[2] = I2C_ReadOneByte(HMC58X3_ADDR, HMC58X3_R_IDC);
}   // getID().

/**************************实现函数********************************************
*函数原型:	  void HMC5883L_SetUp(void)
*功　　能:	   初始化 HMC5883L 使之进入可用状态
输入参数：
输出参数：  无
*******************************************************************************/
void HMC5883L_SetUp(void)
{
	HMC58X3_init(0); // Don't set mode yet, we'll do that later on.
	HMC58X3_setMode(0);
	HMC58X3_setDOR(6);  //75hz 更新率
	HMC58X3_FIFO_init();


}

/**************************实现函数********************************************
*函数原型:	  void HMC5883L_Start_Calib(void)
*功　　能:	   进入磁力计标定
输入参数：
输出参数：  无
*******************************************************************************/
void HMC5883L_Start_Calib(void)
{
	HMC5883_calib = 1; //开始标定
	HMC5883_maxx = 0;	//将原来的标定值清除
	HMC5883_maxy = 0;
	HMC5883_maxz = 0;
	HMC5883_minx = -0;
	HMC5883_miny = -0;
	HMC5883_minz = -0;
}


//------------------End of File----------------------------

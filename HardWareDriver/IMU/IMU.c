/******************** (C) COPYRIGHT 2015 DUT ********************************
 * 作者    ：胡文博
 * 文件名  ：IMU.c
 * 描述    ：姿态解算算法
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

#include "IMU.h"


#define DATA_SIZE 100
volatile float exInt, eyInt, ezInt;  // 误差积分
volatile float integralFBx, integralFBy, integralFBz;
volatile float q0, q1, q2, q3; // 全局四元数
volatile float qa0, qa1, qa2, qa3;
volatile float integralFBhand, handdiff;
volatile double halftime ;
volatile uint32_t lastUpdate, now; // 采样周期计数 单位 us
volatile uint16_t sysytem_time_ms = 0;
volatile float IMU_Pitch, IMU_Roll, IMU_Yaw, ACC_Pitch, ACC_Roll;// 姿态角度 单位 度
volatile float  IMU_GYROx, IMU_GYROy, IMU_GYROz;//姿态角速率 单位 度每秒
volatile unsigned char IMU_inited = 0;
volatile uint16_t imu_clce = 0;
volatile float acc_vector = 0;  //当前加速度感应到的力合  M/S^2
volatile float  acc_X, acc_Y, acc_Z, acc_MX, acc_MY, acc_MZ; //加速度初始化四元数
// Fast inverse square-root
/**************************实现函数********************************************
*函数原型:	   float invSqrt(float x)
*功　　能:	   快速计算 1/Sqrt(x)
输入参数： 要计算的值
输出参数： 结果
*******************************************************************************/
float invSqrt(float x)
{
	volatile float halfx = 0.5f * x;
	volatile float y = x;
	long i = *(long *)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/**************************实现函数********************************************
*函数原型:	   void IMU_init(void)
*功　　能:	  初始化IMU相关
			  初始化各个传感器
			  初始化四元数
			  将积分清零
			  更新系统时间
输入参数：无
输出参数：没有
*******************************************************************************/
void IMU_init(void)
{
	MPU6050_initialize();
	HMC5883L_SetUp();
	delay_ms(50);
	MPU6050_initialize();
	HMC5883L_SetUp();

	// initialize quaternion
	q0 = 1.0f;  //初始化四元数
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	qa0 = 1.0f;  //初始化四元数
	qa1 = 0.0f;
	qa2 = 0.0f;
	qa3 = 0.0f;
	exInt = 0.0;
	eyInt = 0.0;
	ezInt = 0.0;
	integralFBx = 0.0;
	integralFBy = 0.0;
	integralFBz	= 0.0;
	lastUpdate = micros();//更新时间
	now = micros();
}

/**************************实现函数********************************************
*函数原型:	void IMU_getValues(volatile float * values)
*功　　能:	读取加速度 陀螺仪 磁力计 的当前值
输入参数： 	将结果存放的数组首地址
输出参数：没有
*******************************************************************************/
#define new_weight 0.4f//新权重
#define old_weight 0.6f//旧权重

void IMU_getValues(float *values)
{
	int16_t accgyroval[6];
	static  volatile float lastacc[3] = {0, 0, 0};
	int i;
	//读取加速度和陀螺仪的当前ADC
	MPU6050_getMotion6(&accgyroval[0], &accgyroval[1], &accgyroval[2], &accgyroval[3], &accgyroval[4], &accgyroval[5]);
	for(i = 0; i < 6; i++)
	{
		if(i < 3)
		{
			values[i] = (float) accgyroval[i] * new_weight + lastacc[i] * old_weight ;
			lastacc[i] = values[i];
		}
		else
		{
			values[i] = ((float) accgyroval[i]) / 16.4f; //转成度每秒
			//这里已经将量程改成了 2000度每秒  16.4 对应 1度每秒
		}
	}
	HMC58X3_mgetValues(&values[6]);	//读取磁力计的ADC值
	IMU_GYROx = values[3];
	IMU_GYROy = values[4];
	IMU_GYROz = values[5];

}


/**************************实现函数********************************************
*函数原型:	   void IMU_AHRSupdate
*功　　能:	 更新AHRS 更新四元数
输入参数： 当前的测量值。
输出参数：没有
*******************************************************************************/
#define Kp 2.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.03f   // integral gain governs rate of convergence of gyroscope biases

void IMU_AHRSupdate(volatile float gx, volatile float gy, volatile float gz, volatile float ax, volatile float ay, volatile float az, volatile float mx, volatile float my, volatile float mz)
{

	volatile  float norm;
	volatile float hx, hy, hz, bx, bz;
	volatile float vx, vy, vz, wx, wy, wz;
	volatile float ex, ey, ez, halfT;
	volatile float temp0, temp1, temp2, temp3;
	volatile float temp;
	// 先把这些用得到的值算好
	float q0q0 = q0 * q0;
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q0q3 = q0 * q3;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;

	now = micros();  //读取时间
	if(now < lastUpdate)  //定时器溢出过了。
	{
		halfT =  ((float)(now + (0xffffffff - lastUpdate)) / 2000000.0f);
		lastUpdate = now;
		//return ;
	}
	else
	{
		halfT =  ((float)(now - lastUpdate) / 2000000.0f);
	}
	halftime = halfT;
	lastUpdate = now;	//更新时间

	temp = sqrt(ax * ax + ay * ay + az * az);
	temp = (temp / 16384.0f) * 9.8f;   //转成M/S^2为单位的
	acc_vector = acc_vector +   //低通滤波。截止频率20hz
		     (halfT * 2.0f / (7.9577e-3f + halfT * 2.0f)) * (temp - acc_vector);

	norm = invSqrt(ax * ax + ay * ay + az * az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
	//用加速度计算roll、pitch
	//	temp = ax * invSqrt((ay * ay + az * az));
	//	ACC_Pitch = atan(temp)* 57.3;
	//
	//	temp = ay * invSqrt((ax * ax + az * az));
	//	ACC_Roll = atan(temp)* 57.3;

	norm = invSqrt(mx * mx + my * my + mz * mz);
	mx = mx * norm;
	my = my * norm;
	mz = mz * norm;

	// compute reference direction of flux
	//从机体坐标系的电子罗盘测到的矢量转成地理坐标系下的磁场矢量hxyz（测量值）
	hx = 2 * mx * (0.5f - q2q2 - q3q3) + 2 * my * (q1q2 - q0q3) + 2 * mz * (q1q3 + q0q2);
	hy = 2 * mx * (q1q2 + q0q3) + 2 * my * (0.5f - q1q1 - q3q3) + 2 * mz * (q2q3 - q0q1);
	hz = 2 * mx * (q1q3 - q0q2) + 2 * my * (q2q3 + q0q1) + 2 * mz * (0.5f - q1q1 - q2q2);
	/*
	计算地理坐标系下的磁场矢量bxyz（参考值）。
	因为地理地磁水平夹角，我们已知是0度（抛去磁偏角的因素，固定向北），所以by=0，bx=某值
	但地理参考地磁矢量在垂直面上也有分量bz，地球上每个地方都是不一样的。
	我们无法得知，也就无法用来融合（有更适合做垂直方向修正融合的加速度计），所以直接从测量值hz上复制过来，bz=hz。
	磁场水平分量，参考值和测量值的大小应该是一致的(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))。
	因为by=0，所以就简化成(bx*bx)  = ((hx*hx) + (hy*hy))。可算出bx。
	*/
	bx = sqrt((hx * hx) + (hy * hy));
	bz = hz;

	// estimated direction of gravity and flux (v and w)
	/*
	这是把四元数换算成《方向余弦矩阵》中的第三列的三个元素。
	根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。
	所以这里的vx\y\z，其实就是当前的欧拉角（即四元数）的机体坐标参照系上，换算出来的重力单位向量。
	*/
	vx = 2 * (q1q3 - q0q2);
	vy = 2 * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	/*
	我们把地理坐标系上的磁场矢量bxyz，转到机体上来wxyz。
	因为by=0，所以所有涉及到by的部分都被省略了。
	类似上面重力vxyz的推算，因为重力g的gz=1，gx=gy=0，所以上面涉及到gxgy的部分也被省略了
	*/
	wx = 2 * bx * (0.5f - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2);
	wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3);
	wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5f - q1q1 - q2q2);

	// error is sum of cross product between reference direction of fields and direction measured by sensors
	//现在把加速度的测量矢量和参考矢量做叉积，把磁场的测量矢量和参考矢量也做叉积。都拿来来修正陀螺。
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
	/*
	axyz是机体坐标参照系上，加速度计测出来的重力向量，也就是实际测出来的重力向量。
	axyz是测量得到的重力向量，vxyz是陀螺积分后的姿态来推算出的重力向量，它们都是机体坐标参照系上的重力向量。
	那它们之间的误差向量，就是陀螺积分后的姿态和加计测出来的姿态之间的误差。
	向量间的误差，可以用向量叉积（也叫向量外积、叉乘）来表示，exyz就是两个重力向量的叉积。
	这个叉积向量仍旧是位于机体坐标系上的，而陀螺积分误差也是在机体坐标系，而且叉积的大小与陀螺积分误差成正比，正好拿来纠正陀螺。（你可以自己拿东西想象一下）由于陀螺是对机体直接积分，所以对陀螺的纠正量会直接体现在对机体坐标系的纠正。
	*/
	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
		exInt = exInt + ex * Ki * halfT;
		eyInt = eyInt + ey * Ki * halfT;
		ezInt = ezInt + ez * Ki * halfT;

		// adjusted gyroscope measurements
		// 用叉积误差来做PI修正陀螺零偏
		gx = gx + (Kp * ex + exInt);
		gy = gy + (Kp * ey + eyInt);
		gz = gz + (Kp * ez + ezInt);

	}

	// integrate quaternion rate and normalise
	// 四元数微分方程
	temp0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
	temp1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
	temp2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
	temp3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

	// normalise quaternion
	norm = invSqrt(temp0 * temp0 + temp1 * temp1 + temp2 * temp2 + temp3 * temp3);
	q0 = temp0 * norm;
	q1 = temp1 * norm;
	q2 = temp2 * norm;
	q3 = temp3 * norm;
}


#define twoKpDef  (1.0f ) // 2 * proportional gain
#define twoKiDef  (0.2f) // 2 * integral gain

void FreeIMU_AHRSupdate(volatile float gx, volatile float gy, volatile float gz, volatile float ax, volatile float ay, volatile float az)
{
	volatile float norm;
	//  float hx, hy, hz, bx, bz;
	volatile float vx, vy, vz;
	volatile float ex, ey, ez;
	volatile float temp0, temp1, temp2, temp3;

	// 先把这些用得到的值算好
	volatile float q0q0 = qa0 * qa0;
	volatile float q0q1 = qa0 * qa1;
	volatile float q0q2 = qa0 * qa2;
	volatile float q0q3 = qa0 * qa3;
	volatile float q1q1 = qa1 * qa1;
	volatile float q1q2 = qa1 * qa2;
	volatile float q1q3 = qa1 * qa3;
	volatile float q2q2 = qa2 * qa2;
	volatile float q2q3 = qa2 * qa3;
	volatile float q3q3 = qa3 * qa3;

	norm = invSqrt(ax * ax + ay * ay + az * az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

	// estimated direction of gravity and flux (v and w)
	vx = 2 * (q1q3 - q0q2);
	vy = 2 * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;

	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ay * vz - az * vy) ;
	ey = (az * vx - ax * vz) ;
	ez = (ax * vy - ay * vx) ;

	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{

		integralFBx +=  ex * twoKiDef * halftime;
		integralFBy +=  ey * twoKiDef * halftime;
		integralFBz +=  ez * twoKiDef * halftime;

		gx = gx + twoKpDef * ex + integralFBx;
		gy = gy + twoKpDef * ey + integralFBy;
		gz = gz + twoKpDef * ez + integralFBz;

	}
	// integrate quaternion rate and normalise
	temp0 = qa0 + (double)(-qa1 * gx - qa2 * gy - qa3 * gz) * halftime;
	temp1 = qa1 + (double)(qa0 * gx + qa2 * gz - qa3 * gy) * halftime;
	temp2 = qa2 + (double)(qa0 * gy - qa1 * gz + qa3 * gx) * halftime;
	temp3 = qa3 + (double)(qa0 * gz + qa1 * gy - qa2 * gx) * halftime;

	// normalise quaternion
	norm = invSqrt(temp0 * temp0 + temp1 * temp1 + temp2 * temp2 + temp3 * temp3);
	qa0 = temp0 * norm;
	qa1 = temp1 * norm;
	qa2 = temp2 * norm;
	qa3 = temp3 * norm;
}

/**************************实现函数********************************************
*函数原型:	   void IMU_getQ(float * q)
*功　　能:	 更新四元数 返回当前的四元数组值
输入参数： 将要存放四元数的数组首地址
输出参数：没有
*******************************************************************************/
float mygetqval[9];	//用于存放传感器转换结果的数组
void IMU_getQ(float *q)
{

	IMU_getValues(mygetqval);
	//将陀螺仪的测量值转成弧度每秒
	//加速度和磁力计保持 ADC值　不需要转换
	IMU_AHRSupdate(mygetqval[3] * M_PI / 180, mygetqval[4] * M_PI / 180, mygetqval[5] * M_PI / 180,
		       mygetqval[0], mygetqval[1], mygetqval[2], mygetqval[6], mygetqval[7], mygetqval[8]);

	FreeIMU_AHRSupdate(mygetqval[3] * M_PI / 180, mygetqval[4] * M_PI / 180, mygetqval[5] * M_PI / 180,
			   mygetqval[0], mygetqval[1], mygetqval[2]);

	q[0] = qa0; //返回当前值	FreeIMU_AHRSupdate 计算出来的四元数 被用到
	q[1] = qa1;
	q[2] = qa2;
	q[3] = qa3;
}

// a varient of asin() that checks the input ranges and ensures a
// valid angle as output. If nan is given as input then zero is
// returned.
float safe_asin(float v)
{
	if (isnan(v))
	{
		return 0.0f;
	}
	if (v >= 1.0f)
	{
		return M_PI / 2;
	}
	if (v <= -1.0f)
	{
		return -M_PI / 2;
	}
	return asin(v);
}


/**************************实现函数********************************************
*函数原型:	   void IMU_getYawPitchRoll(float * angles)
*功　　能:	 更新四元数 返回当前解算后的姿态数据
输入参数： 将要存放姿态角的数组首地址
输出参数：没有
*******************************************************************************/
void IMU_getYawPitchRoll(float *angles)
{
	static float q[4]; //　四元数
	IMU_getQ(q); //更新全局四元数
	//	angles[2] = IMU_Roll = ACC_Roll;
	//	angles[1] = IMU_Pitch = ACC_Pitch;
	//  angles[0] = IMU_Yaw;
	IMU_Roll = angles[2] = (atan2(2.0f * (q[0] * q[1] + q[2] * q[3]),
				      1 - 2.0f * (q[1] * q[1] + q[2] * q[2]))) * 180 / M_PI;
	// we let safe_asin() handle the singularities near 90/-90 in pitch
	IMU_Pitch = angles[1] = -safe_asin(2.0f * (q[0] * q[2] - q[3] * q[1])) * 180 / M_PI;

	IMU_Yaw = angles[0] = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / M_PI; // yaw

	//	if(IMU_Yaw <0)IMU_Yaw +=360.0f;  //将 -+180度  转成0-360度
}
/**************************实现函数********************************************
*函数原型:	   void Initialize_Q(void)
*功　　能:  用加速度和罗盘初始化四元数
输入参数：没有
输出参数：没有
*******************************************************************************/
static float acc[9];
void Initialize_Q()
{
	int i;
	volatile float temp, roll, pitch, yaw, yh, xh;
	for(i = 0; i < DATA_SIZE; i++)
	{
		IMU_getValues(acc);
		acc_X += acc[0];
		acc_Y += acc[1];
		acc_Z += acc[2];
		acc_MX += acc[6];
		acc_MY += acc[7];
		acc_MZ += acc[8];
	}
	acc_X /= DATA_SIZE;
	acc_Y /= DATA_SIZE;
	acc_Z /= DATA_SIZE;
	acc_MX /= DATA_SIZE;
	acc_MY /= DATA_SIZE;
	acc_MZ /= DATA_SIZE;

	temp = acc_X * invSqrt((acc_Y * acc_Y + acc_Z * acc_Z));
	pitch = atan(temp) * 57.3;

	temp = acc_Y * invSqrt((acc_X * acc_X + acc_Z * acc_Z));
	roll = atan(temp) * 57.3;

	yh = acc_MY * cos(roll) + acc_MZ * sin(roll);
	xh = acc_MX * cos(pitch) + acc_MY * sin(roll) * sin(pitch) - acc_MZ * cos(roll) * sin(pitch);
	yaw = atan2(yh, xh);
	//初始化四元数，欧拉角转四元数
	q0 = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
	q1 = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
	q2 = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
	q3 = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
}

//------------------End of File----------------------------

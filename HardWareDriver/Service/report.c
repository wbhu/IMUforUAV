/******************** (C) COPYRIGHT 2015 DUT ********************************
 * 作者    ：胡文博
 * 文件名  ：report.c
 * 描述    ：上位机上传数据
 * 日期    ：2015/11/30 12:43:38
 * 联系方式：1461318172（qq）
**********************************************************************************/


#include "common.h"
#include "usart.h"


//串口1发送1个字符
//c:要发送的字符
void usart1_send_char(u8 c)
{

	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	USART_SendData(USART1, c);

}
//智能车上位机数据上传函数
void Report_imu( unsigned short int roll,  unsigned short int pitch,  unsigned short int yaw)
{
	int i, j;
	static unsigned short int send_data[3][8] = { { 0 }, { 0 }, { 0 } };

	send_data[0][0] = ( unsigned short int)(roll);
	send_data[0][1] = ( unsigned short int)(pitch);
	send_data[0][2] = ( unsigned short int)(yaw);
	send_data[0][3] = (unsigned short int)(0);
	send_data[0][4] = (unsigned short int)(0);
	send_data[0][5] = (unsigned short int)(0);
	send_data[0][6] = (unsigned short int)(0);
	send_data[0][7] = (unsigned short int)(0);

	send_data[1][0] = (unsigned short int)(0);
	send_data[1][1] = (unsigned short int)(0);
	send_data[1][2] = (unsigned short int)(0);
	send_data[1][3] = (unsigned short int)(0);
	send_data[1][4] = (unsigned short int)(0);
	send_data[1][5] = (unsigned short int)(0);
	send_data[1][6] = (unsigned short int)(0);
	send_data[1][7] = (unsigned short int)(0);

	send_data[2][0] = (unsigned short int)(0);
	send_data[2][1] = (unsigned short int)(0);
	send_data[2][2] = (unsigned short int)(0);
	send_data[2][3] = (unsigned short int)(0);
	send_data[2][4] = (unsigned short int)(0);
	send_data[2][5] = (unsigned short int)(0);
	send_data[2][6] = (unsigned short int)(0);
	send_data[2][7] = (unsigned short int)(0);

	printf("ST");
	for ( i = 0; i < 3; i++)
		for ( j = 0; j < 8; j++)
		{
			usart1_send_char((unsigned char)(send_data[i][j] & 0x00ff));
			usart1_send_char((unsigned char)(send_data[i][j] >> 8u));
		}
}

//void usart1_send_char(u8 c)
//{

//	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
//	USART_SendData(USART1, c);

//}
//传送数据给匿名四轴上位机软件(V2.6版本)
//fun:功能字. 0XA0~0XAF
//data:数据缓存区,最多28字节!!
//len:data区有效数据个数
//void usart1_niming_report(u8 fun, u8 *data, u8 len)
//{
//	u8 send_buf[32];
//	u8 i;
//	if(len > 28)return;	//最多28字节数据
//	send_buf[len + 3] = 0;	//校验数置零
//	send_buf[0] = 0X88;	//帧头
//	send_buf[1] = fun;	//功能字
//	send_buf[2] = len;	//数据长度
//	for(i = 0; i < len; i++)send_buf[3 + i] = data[i];			//复制数据
//	for(i = 0; i < len + 3; i++)send_buf[len + 3] += send_buf[i];	//计算校验和
//	for(i = 0; i < len + 4; i++)usart1_send_char(send_buf[i]);	//发送数据到串口1
//}
//发送加速度传感器数据和陀螺仪数据
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
//void mpu6050_send_data(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz)
//{
//	u8 tbuf[12];
//	tbuf[0] = (aacx >> 8) & 0XFF;
//	tbuf[1] = aacx & 0XFF;
//	tbuf[2] = (aacy >> 8) & 0XFF;
//	tbuf[3] = aacy & 0XFF;
//	tbuf[4] = (aacz >> 8) & 0XFF;
//	tbuf[5] = aacz & 0XFF;
//	tbuf[6] = (gyrox >> 8) & 0XFF;
//	tbuf[7] = gyrox & 0XFF;
//	tbuf[8] = (gyroy >> 8) & 0XFF;
//	tbuf[9] = gyroy & 0XFF;
//	tbuf[10] = (gyroz >> 8) & 0XFF;
//	tbuf[11] = gyroz & 0XFF;
//	usart1_niming_report(0XA1, tbuf, 12); //自定义帧,0XA1
//}
//通过串口1上报结算后的姿态数据给电脑
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
//roll:横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
//pitch:俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
//yaw:航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
//void usart1_report_imu(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz, short roll, short pitch, short yaw)
//{
//	u8 tbuf[28];
//	u8 i;
//	for(i = 0; i < 28; i++)tbuf[i] = 0; //清0
//	tbuf[0] = (aacx >> 8) & 0XFF;
//	tbuf[1] = aacx & 0XFF;
//	tbuf[2] = (aacy >> 8) & 0XFF;
//	tbuf[3] = aacy & 0XFF;
//	tbuf[4] = (aacz >> 8) & 0XFF;
//	tbuf[5] = aacz & 0XFF;
//	tbuf[6] = (gyrox >> 8) & 0XFF;
//	tbuf[7] = gyrox & 0XFF;
//	tbuf[8] = (gyroy >> 8) & 0XFF;
//	tbuf[9] = gyroy & 0XFF;
//	tbuf[10] = (gyroz >> 8) & 0XFF;
//	tbuf[11] = gyroz & 0XFF;
//	tbuf[18] = (roll >> 8) & 0XFF;
//	tbuf[19] = roll & 0XFF;
//	tbuf[20] = (pitch >> 8) & 0XFF;
//	tbuf[21] = pitch & 0XFF;
//	tbuf[22] = (yaw >> 8) & 0XFF;
//	tbuf[23] = yaw & 0XFF;
//	usart1_niming_report(0XAF, tbuf, 28); //飞控显示帧,0XAF
//}

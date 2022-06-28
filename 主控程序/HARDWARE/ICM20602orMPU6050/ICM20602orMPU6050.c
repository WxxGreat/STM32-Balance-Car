#include "ICM20602orMPU6050.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"

//初始化ICM20602 or MPU6050
//返回值:0,成功
//其他,错误代码
uint8_t MPU_Init(void)
{
	uint8_t res;
	IIC_Init();								 //初始化IIC总线
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X80); //复位MPU6050
	delay_ms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X00); //唤醒MPU6050
	MPU_Set_Gyro_Fsr(3);					 //陀螺仪传感器,±2000dps
	MPU_Set_Accel_Fsr(0);					 //加速度传感器,±2g
	MPU_Set_Rate(1000);						 //设置采样率1000Hz
	MPU_Write_Byte(MPU_INT_EN_REG, 0X00);	 //关闭所有中断
	MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00); // I2C主模式关闭
	MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00);	 //关闭FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X80); // INT引脚低电平有效
	res = MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if (res == MPU_ADDR || res == 0x12) //器件ID正确
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X01); //设置CLKSEL,PLL X轴为参考
		MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0X00); //加速度与陀螺仪都工作
		MPU_Set_Rate(200);						 //设置采样率为100Hz
	}
	else  return 1;
	return 0;
}
//设置CM20602 or MPU6050陀螺仪传感器满量程范围
// fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//其他,设置失败
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3); //设置陀螺仪满量程范围
}



//设置CM20602 or MPU6050加速度传感器满量程范围
// fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//其他,设置失败
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3); //设置加速度传感器满量程范围
}



//设置CM20602 or MPU6050的数字低通滤波器
// lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
// 其他,设置失败
uint8_t MPU_Set_LPF(u16 lpf)
{
	uint8_t data = 0;
	if (lpf >= 188)     data = 1;
	else if (lpf >= 98) data = 2;
	else if (lpf >= 42) data = 3;
	else if (lpf >= 20) data = 4;
	else if (lpf >= 10) data = 5;
	else                data = 6;
	return MPU_Write_Byte(MPU_CFG_REG, data); //设置数字低通滤波器
}



//设置CM20602 or MPU6050的采样率(假定Fs=1KHz)
// rate:4~1000(Hz)
//返回值:0,设置成功
//其他,设置失败
uint8_t MPU_Set_Rate(u16 rate)
{
	uint8_t data;
	if (rate > 1000)  rate = 1000;
	if (rate < 4)     rate = 4;
	data = 1000 / rate - 1;
	data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data); //设置数字低通滤波器
	return MPU_Set_LPF(rate / 2);					  //自动设置LPF为采样率的一半
}

/**
 * @brief    均值滤波获得温度值
 * 
 * @param    无
 * 
 * @return   温度值
 * 
 * @note     使用模拟FIFO求10次采样的平均值
 * 
 * @example  MPU_Get_Temperature(&MPU6050_Temperature); 变量MPU6050_Temperature即为测得的温度。
 * 
 * @date     2022/6/21 16:15
 * 
 * @author   WxxGreat
 */
#define Temperature_FIFO_SIZE 10 //采样数据个数
static short Temperature_FIFO[Temperature_FIFO_SIZE] = {0};
void MPU_Get_Temperature(float *MPU6050_Temperature)
{
	static unsigned char MPU6050_Temperature_Index;

	uint8_t buf[2];
	uint8_t i;
	MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf); //读温度寄存器高8位和第8位

	Temperature_FIFO[MPU6050_Temperature_Index] = ((unsigned short int)buf[0] << 8) | buf[1]; //将读到的温度值放到FIFO队列中
	MPU6050_Temperature_Index = (MPU6050_Temperature_Index + 1) % Temperature_FIFO_SIZE;	  //更新下一个值进入FIFO队列中的位置

	for (i = 0; i < Temperature_FIFO_SIZE; i++)
		*MPU6050_Temperature += Temperature_FIFO[i]; //对FIFO中所有的值累加

	*MPU6050_Temperature = *MPU6050_Temperature / Temperature_FIFO_SIZE / 340 + 36.53f;
}



//得到陀螺仪值(原始值)
// gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//其他,错误代码
void MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
	uint8_t buf[6];
  MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);

	*gx = ((u16)buf[0] << 8) | buf[1];
	*gy = ((u16)buf[2] << 8) | buf[3];
	*gz = ((u16)buf[4] << 8) | buf[5];
}


//得到加速度值(原始值)
// gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//其他,错误代码
void MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
	uint8_t buf[6];
	MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);

	*ax = ((u16)buf[0] << 8) | buf[1];
	*ay = ((u16)buf[2] << 8) | buf[3];
	*az = ((u16)buf[4] << 8) | buf[5];
}


// IIC连续写
// addr:器件地址
// reg:寄存器地址
// len:写入长度
// buf:数据区
//返回值:0,正常
//其他,错误代码
uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	uint8_t i;
	IIC_Start();
	IIC_Send_Byte((addr << 1) | 0); //发送器件地址+写命令
	if (IIC_Wait_Ack())				//等待应答
	{
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg); //写寄存器地址
	IIC_Wait_Ack();		//等待应答
	for (i = 0; i < len; i++)
	{
		IIC_Send_Byte(buf[i]); //发送数据
		if (IIC_Wait_Ack())	   //等待ACK
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_Stop();
	return 0;
}


// IIC连续读
// addr:器件地址
// reg:要读取的寄存器地址
// len:要读取的长度
// buf:读取到的数据存储区
//返回值:0,正常
//其他,错误代码
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	IIC_Start();
	IIC_Send_Byte((addr << 1) | 0); //发送器件地址+写命令
	if (IIC_Wait_Ack())				//等待应答
	{
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg); //写寄存器地址
	IIC_Wait_Ack();		//等待应答
	IIC_Start();
	IIC_Send_Byte((addr << 1) | 1); //发送器件地址+读命令
	IIC_Wait_Ack();					//等待应答
	while (len)
	{
		if (len == 1)
			*buf = IIC_Read_Byte(0); //读数据,发送nACK
		else
			*buf = IIC_Read_Byte(1); //读数据,发送ACK
		len--;
		buf++;
	}
	IIC_Stop(); //产生一个停止条件
	return 0;
}


// IIC写一个字节
// reg:寄存器地址
// data:数据
//返回值:0,正常
//其他,错误代码
uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data)
{
	IIC_Start();
	IIC_Send_Byte((MPU_ADDR << 1) | 0); //发送器件地址+写命令
	if (IIC_Wait_Ack())					//等待应答
	{
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg);	 //写寄存器地址
	IIC_Wait_Ack();		 //等待应答
	IIC_Send_Byte(data); //发送数据
	if (IIC_Wait_Ack())	 //等待ACK
	{
		IIC_Stop();
		return 1;
	}
	IIC_Stop();
	return 0;
}


// IIC读一个字节
// reg:寄存器地址
//返回值:读到的数据
uint8_t MPU_Read_Byte(uint8_t reg)
{
	uint8_t res;
	IIC_Start();
	IIC_Send_Byte((MPU_ADDR << 1) | 0); //发送器件地址+写命令
	IIC_Wait_Ack();						//等待应答
	IIC_Send_Byte(reg);					//写寄存器地址
	IIC_Wait_Ack();						//等待应答
	IIC_Start();
	IIC_Send_Byte((MPU_ADDR << 1) | 1); //发送器件地址+读命令
	IIC_Wait_Ack();						//等待应答
	res = IIC_Read_Byte(0);				//读取数据,发送nACK
	IIC_Stop();							//产生一个停止条件
	return res;
}

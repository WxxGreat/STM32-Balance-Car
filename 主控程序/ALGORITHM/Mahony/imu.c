#include "imu.h"

#define OFFSET_COUNT 100
#define FIFO_SIZE 10

signed short     int IMU_FIFO[6][FIFO_SIZE];
static unsigned char Wr_Index = 0; // FIFO队列计数器

static float Pitch_offset;
static float Roll_offset;
static float Yaw_offset;
short aacx,aacy,aacz;		//加速度传感器原始数据 
short gyrox,gyroy,gyroz;	//陀螺仪原始数据 
float IMU_Temperature = 0 ; //IMU获得的温度	

IMU_Info imu;



/**-------------------------------------------------------------------------------------------------------------------
  * @brief    将最新的数据放入FIFO队列
  *
  * @param    IMU_FIFO:   
	*           IMU_FIFO[0][]:GyroX的FIFO队列
  *           IMU_FIFO[1][]:GyroY的FIFO队列
  *           IMU_FIFO[2][]:GyroZ的FIFO队列
  *           IMU_FIFO[3][]:AcceX的FIFO队列
  *           IMU_FIFO[4][]:AcceY的FIFO队列
  *           IMU_FIFO[5][]:AcceZ的FIFO队列
	*
  * @param 	  IMU_val :
	*           IMU获得的一个最新值
  *
  * @return   NULL
  *
  * @date     2022/6/19 17:35
  *
  * @author   WxxGreat
  *-----------------------------------------------------------------------------------------------------------------*/
void IMU_NewVal(int16_t *IMU_FIFO, int16_t IMU_val)
{
	IMU_FIFO[Wr_Index] = IMU_val;
}




/**-------------------------------------------------------------------------------------------------------------------
  * @brief    将FIFO队列中的数据取平均值
  *
  * @param    IMU_FIFO[0][]:GyroX的FIFO队列
  *           IMU_FIFO[1][]:GyroY的FIFO队列
  *           IMU_FIFO[2][]:GyroZ的FIFO队列
  *           IMU_FIFO[3][]:AcceX的FIFO队列
  *           IMU_FIFO[4][]:AcceY的FIFO队列
  *           IMU_FIFO[5][]:AcceZ的FIFO队列
  *
  * @return   经过均值滤波的6轴数据
  *
  * @date     2022/6/20 14:28
  *
  * @author   WxxGreat
  *-----------------------------------------------------------------------------------------------------------------*/
int16_t IMU_GetAvg(int16_t *IMU_FIFO)
{
	u8 i;
	int32_t sum = 0;
	for (i = 0; i < FIFO_SIZE; i++)
		sum += IMU_FIFO[i];
	sum = sum / FIFO_SIZE;
	return (int16_t)sum;
}




/**-------------------------------------------------------------------------------------------------------------------
  * @brief    获得经过均值滤波后的IMU 6轴原始值
  *
  * @param    IMU_values[0]:GyroX
  *           IMU_values[1]:GyroY
  *           IMU_values[2]:GyroZ
  *           IMU_values[3]:AcceX
  *           IMU_values[4]:AcceY
  *           IMU_values[5]:AcceZ
  *
  * @return   6轴数据原始值
  *
  * @date     2022/6/23 16:37
  *
  * @author   WxxGreat
  *-----------------------------------------------------------------------------------------------------------------*/
 void Get_IMU_Values(float *IMU_values)
{
	short i = 0;
	int16_t gyro[3], acc[3];

	IMU_readGyro_Acc(&gyro[0], &acc[0]);
	
#if USE_ICM42605 == 1
	    Get_ICM42605_Tempdata(&IMU_Temperature);
#else
      MPU_Get_Temperature(&IMU_Temperature);
#endif
	
	for (; i < 3; i++)
	{
		IMU_values[i] = ((float)gyro[i]) / 16.4f; // gyro range: +-2000; adc accuracy 16 bits: 2^16=65536; 65536/4000=16.4; so  1^-> 16.4
		IMU_values[3 + i] = (float)acc[i];
	}
}



/**-------------------------------------------------------------------------------------------------------------------
  * @brief    读取6轴原始值并进行均值滤波
  *
  * @param    gyro[]: 经过均值滤波的角速度值地址
  * @param    acce[]: 经过均值滤波的加速度值地址
  *
  * @return   经过均值滤波的6轴数据
  *
  * @date     2022/6/22 15:46
  *
  * @author   WxxGreat
  *-----------------------------------------------------------------------------------------------------------------*/
void IMU_readGyro_Acc(int16_t *gyro, int16_t *acce)
{
	static short buf[6];
	static int16_t gx, gy, gz;
	static int16_t ax, ay, az;

#if USE_ICM42605 == 1
	Get_ICM42605_gyro(&buf[0], &buf[1], &buf[2]);
	Get_ICM42605_accdata(&buf[3], &buf[4], &buf[5]);
#else
	MPU_Get_Gyroscope(&buf[0], &buf[1], &buf[2]);
	MPU_Get_Accelerometer(&buf[3], &buf[4], &buf[5]);
#endif
	
	IMU_NewVal(&IMU_FIFO[0][0], buf[0]);
	IMU_NewVal(&IMU_FIFO[1][0], buf[1]);
	IMU_NewVal(&IMU_FIFO[2][0], buf[2]);

	IMU_NewVal(&IMU_FIFO[3][0], buf[3]);
	IMU_NewVal(&IMU_FIFO[4][0], buf[4]);
	IMU_NewVal(&IMU_FIFO[5][0], buf[5]);

	Wr_Index = (Wr_Index + 1) % FIFO_SIZE;

	gx = IMU_GetAvg(&IMU_FIFO[0][0]);
	gy = IMU_GetAvg(&IMU_FIFO[1][0]);
	gz = IMU_GetAvg(&IMU_FIFO[2][0]);

	gyro[0] = gx - Roll_offset;
	gyro[1] = gy - Pitch_offset;
	gyro[2] = gz - Yaw_offset;

	ax = IMU_GetAvg(&IMU_FIFO[3][0]);
	ay = IMU_GetAvg(&IMU_FIFO[4][0]);
	az = IMU_GetAvg(&IMU_FIFO[5][0]);

	acce[0] = ax;
	acce[1] = ay;
	acce[2] = az;
	
}


/**-------------------------------------------------------------------------------------------------------------------
  * @brief    最终在定时器中调用的函数
  *
  * @param    NULL
  *
  * @return   Roll和Pitch
  *
  * @date     2022/6/23 17:02
  *
  * @author   WxxGreat
  *-----------------------------------------------------------------------------------------------------------------*/
void IMU_Update(void)
{
	static float q[4];
	float Values[6];
	Get_IMU_Values(Values);

	//将角度更改为弧度，使用Mahony计算
	MahonyAHRSupdateIMU(Values[0] * 3.14159265358979f / 180, Values[1] * 3.14159265358979f / 180, Values[2] * 3.14159265358979f / 180,Values[3], Values[4], Values[5]);

	//保存四元数
	q[0] = q0;
	q[1] = q1;
	q[2] = q2;
	q[3] = q3;

	imu.ax = Values[3];
	imu.ay = Values[4];
	imu.az = Values[5];

	imu.Pitch_v = Values[0];
	imu.Roll_v = Values[1];
	imu.Yaw_v = Values[2];

  //四元数法计算欧拉角
	imu.Roll = (atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), 1 - 2.0f * (q[1] * q[1] + q[2] * q[2]))) * 180 / 3.14159265358979f;
//imu.Pitch = -safe_asin(2.0f * (q[0] * q[2] - q[1] * q[3])) * 180 / 3.14159265358979f;

}



/**-------------------------------------------------------------------------------------------------------------------
  * @brief    获得IMU的零点偏移
  *
  * @note     使用Mahony算法去除IMU的零偏以获得最好的估计值
  *
  * @return   Roll_offset ，Pitch_offset  ，Yaw_offset
  *
  * @date     2022/6/19 17:00
  *
  * @author   WxxGreat
  *-----------------------------------------------------------------------------------------------------------------*/
void IMU_Init_Offset(void)
{
	short i;
	int tempgx = 0, tempgy = 0, tempgz = 0;
	int tempax = 0, tempay = 0, tempaz = 0;
	Pitch_offset = 0;
	Roll_offset = 0;
	Yaw_offset = 0;
  delay_ms(10);
	
	// read the mpu data for calculate the offset
	for (i = 0; i < OFFSET_COUNT; i++)
	{
		delay_ms(5);
#if USE_ICM42605 == 1
		Get_ICM42605_accdata(&aacx,&aacy,&aacz);  //得到加速度传感器数据
    Get_ICM42605_gyro(&gyrox,&gyroy,&gyroz);  //得到陀螺仪数据
	  Get_ICM42605_Tempdata(&IMU_Temperature);
#else
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
    MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
		MPU_Get_Temperature(&IMU_Temperature); 
#endif
		tempgx += gyrox;
		tempgy += gyroy;
		tempgz += gyroz;

		tempax += aacx;
		tempay += aacy;
		tempaz += aacz;
	}
		
	Roll_offset  = tempgx / OFFSET_COUNT;
	Pitch_offset = tempgy / OFFSET_COUNT;
	Yaw_offset   = tempgz / OFFSET_COUNT;

	//printf("Pitch_offset = %f,Roll_offset = %f,Yaw_offset = %f\r\n", Pitch_offset, Roll_offset, Yaw_offset);
}



/**-------------------------------------------------------------------------------------------------------------------
  * @brief    IMU初始化函数
  *
  * @note     要用ICM42605就在Keil5:  "魔法棒 -> C/C++ -> Define"   使得USE_ICM42605 = 1
  *
  * @return   NULL
  *
  * @date     2022/6/19 17:00
  *
  * @author   WxxGreat
  *-----------------------------------------------------------------------------------------------------------------*/
void IMU_Init(void)
{
 	char Err;
	MPU_RE_INIT:
#if  USE_ICM42605 == 1
	Err = ICM42605_init();
#else
	Err = MPU_Init();
#endif
	if (Err != 0)
	{
		LCD_printf(0,90,"IMU error:%d",Err);
		goto MPU_RE_INIT;
	}
	
	delay_ms(10);
	
	LCD_ShowChinese(0,90,"陀螺仪校准中",0xFFFF,0x0000,32,0);
	LCD_ShowChar(187,90,':',0xFFFF,0x0000,32,0);//显示一个字符
	
	IMU_Init_Offset();
}


//DeBug函数
IMU_Info *IMU_GetInfo(void) { return &imu; }

//arcsin函数
float safe_asin(float v)
{
	if (isnan(v))  return 0.0f;
	if (v >= 1.0f) return 3.14159265358979f/2;
	if (v <= -1.0f)return -3.14159265358979f/2;
	return asin(v);
}


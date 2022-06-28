#include "i2c.h"
#include "ICM42605.h"
#include "stdio.h"

/**
 * @brief          ICM42605使用模拟IIC通讯驱动
 *
 * @note           
 *
 * @date           2022-6-10 18:22
 *
 * @author         WxxGreat
 *
 * @version        1.0   
 */


static float accSensitivity;   		/* 加速度的最小分辨率 */
static float gyroSensitivity;    	/* 陀螺仪的最小分辨率 */





/**
 * @brief          将原始ICM42605的值转化为有物理意义的加速度(单位m/s^2)和角速度(单位rad/s)
 *
 * @param          NULL
 *
 * @return         void
 *
 * @usage          调用该函数前，先读取原始值;根据传感器寄存器配置调整本函数数值
 *
 * @date           2022-6-10 18:22
 *
 * @author         WxxGreat
 *
 * @version        1.0   
 */
void ICM42605_AD_trun(void)
{
	  float g = 9.7947f; //合肥重力加速度
    float pi = 3.14159265359f;
		Get_ICM42605_accdata(&aacx,&aacy,&aacz);
	  Get_ICM42605_gyro(&gyrox,&gyroy,&gyroz);
    aacx =  aacx * (8.0f * g / 32768.0f);//转化后,单位m/s^2
    aacy =  aacy * (8.0f * g / 32768.0f);
    aacz =  aacz * (8.0f * g / 32768.0f);
    gyrox = gyrox * (2000.0f * pi / 180 / 32768.0f);//转化后,单位rad/s
    gyroy = gyroy * (2000.0f * pi / 180 / 32768.0f);
    gyroz = gyroz * (2000.0f * pi / 180 / 32768.0f);
//  temp_data = (icm4_temp / 132.48) + 25;//转化后,单位摄氏度，To fix
}

//-------------------------------------------------------------------------------------------------------------------
//  使用软件IIC通信
//-------------------------------------------------------------------------------------------------------------------

/**
 * @brief          自检ICM42605
 *
 * @param          NULL
 *
 * @return         void
 *
 * @usage          调用该函数前，请先调用模拟IIC的初始化
 *
 * @date           2022-6-10 18:22
 *
 * @author         WxxGreat
 *
 * @version        1.0   
 */
void ICM42605_self1_check(void)
{
    uint8_t dat=0;
    while(0x42 != dat)   //读取ICM42605 ID, 默认ID是0x42
    {
        dat = ICM42605_Read_Byte(ICM42605_WHO_AM_I);
        delay_ms(1);
    }
}

/* 计算当前加速度分辨率 */
float bsp_Icm42605GetAres(uint8_t Ascale)
{
    switch(Ascale)
    {
    case AFS_2G:
        accSensitivity = 2000 / 32768.0f;
        break;
    case AFS_4G:
        accSensitivity = 4000 / 32768.0f;
        break;
    case AFS_8G:
        accSensitivity = 8000 / 32768.0f;
        break;
    case AFS_16G:
        accSensitivity = 16000 / 32768.0f;
        break;
    }

    return accSensitivity;
}

/* 计算当前陀螺仪分辨率 */
float bsp_Icm42605GetGres(uint8_t Gscale)
{
    switch(Gscale)
    {
    case GFS_15_125DPS:
        gyroSensitivity = 15.125f / 32768.0f;
        break;
    case GFS_31_25DPS:
        gyroSensitivity = 31.25f / 32768.0f;
        break;
    case GFS_62_5DPS:
        gyroSensitivity = 62.5f / 32768.0f;
        break;
    case GFS_125DPS:
        gyroSensitivity = 125.0f / 32768.0f;
        break;
    case GFS_250DPS:
        gyroSensitivity = 250.0f / 32768.0f;
        break;
    case GFS_500DPS:
        gyroSensitivity = 500.0f / 32768.0f;
        break;
    case GFS_1000DPS:
        gyroSensitivity = 1000.0f / 32768.0f;
        break;
    case GFS_2000DPS:
        gyroSensitivity = 2000.0f / 32768.0f;
        break;
    }
	
	gyroSensitivity = gyroSensitivity*1000;
	
	
    return gyroSensitivity;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      初始化ICM40605
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               调用该函数前，请先调用模拟IIC的初始化
//-------------------------------------------------------------------------------------------------------------------
char ICM42605_init(void)
{
    uint8_t reg_val;

    IIC_Init();								 //初始化IIC总线
    delay_ms(10);  //上电延时

    
    ICM42605_self1_check();
	
    if(ICM42605_Read_Byte(ICM42605_WHO_AM_I) == 0x42)//自检
		{   
			  //卡在这里原因有以下几点
        //1 ICM42605坏了，如果是新的这样的概率极低
        //2 接线错误或者没有接好
        //3 可能你需要外接上拉电阻，上拉到3.3V
	
				
			ICM42605_Write_Byte(ICM42605_REG_BANK_SEL, 0); 		/* 设置bank 0区域寄存器 */
			ICM42605_Write_Byte(ICM42605_REG_BANK_SEL, 0x01); 	/* 软复位传感器 */
			
			delay_ms(10);//至少等待1ms

				
			ICM42605_Write_Byte(ICM42605_REG_BANK_SEL, 1); 		/* 设置bank 1区域寄存器 */
			ICM42605_Write_Byte(ICM42605_INTF_CONFIG4, 0x02); 	/* 设置为4线SPI通信 */

			ICM42605_Write_Byte(ICM42605_REG_BANK_SEL, 0); 		/* 设置bank 0区域寄存器 */
			ICM42605_Write_Byte(ICM42605_FIFO_CONFIG, 0x40); 	/* Stream-to-FIFO Mode(page61) */


			reg_val = ICM42605_Read_Byte(ICM42605_INT_SOURCE0);

			ICM42605_Write_Byte(ICM42605_INT_SOURCE0, 0x00);
			ICM42605_Write_Byte(ICM42605_FIFO_CONFIG2, 0x00); 
			ICM42605_Write_Byte(ICM42605_FIFO_CONFIG3, 0x02); 
			ICM42605_Write_Byte(ICM42605_INT_SOURCE0, reg_val);
			ICM42605_Write_Byte(ICM42605_FIFO_CONFIG1, 0x63); 	/* 使能FIFO */

			ICM42605_Write_Byte(ICM42605_REG_BANK_SEL, 0x00);
			ICM42605_Write_Byte(ICM42605_INT_CONFIG, 0x36);

			ICM42605_Write_Byte(ICM42605_REG_BANK_SEL, 0x00);
			reg_val = ICM42605_Read_Byte(ICM42605_INT_SOURCE0);
			reg_val |= (1 << 2);
			ICM42605_Write_Byte(ICM42605_INT_SOURCE0, reg_val);

			bsp_Icm42605GetAres(AFS_2G);						/* 默认8g量程 */
			ICM42605_Write_Byte(ICM42605_REG_BANK_SEL, 0x00);
			reg_val = ICM42605_Read_Byte(ICM42605_ACCEL_CONFIG0);
			reg_val |= (AFS_2G << 5);  
			reg_val |= (AODR_1000Hz);    
			ICM42605_Write_Byte(ICM42605_ACCEL_CONFIG0, reg_val);

			bsp_Icm42605GetGres(GFS_2000DPS);					/* 默认1000dps量程 */
			ICM42605_Write_Byte(ICM42605_REG_BANK_SEL, 0x00);
			reg_val = ICM42605_Read_Byte(ICM42605_GYRO_CONFIG0);	
			reg_val |= (GFS_2000DPS << 5);  
			reg_val |= (AODR_1000Hz);     
			ICM42605_Write_Byte(ICM42605_GYRO_CONFIG0, reg_val);

			ICM42605_Write_Byte(ICM42605_REG_BANK_SEL, 0x00);
			reg_val = ICM42605_Read_Byte(ICM42605_PWR_MGMT0); 	/* 读取PWR—MGMT0当前寄存器的值(page72) */
			reg_val &= ~(1 << 5);		/* 使能温度测量 */
			reg_val |= ((3) << 2);		/* 设置GYRO_MODE  0:关闭 1:待机 2:预留 3:低噪声 */
			reg_val |= (3);				/* 设置ACCEL_MODE 0:关闭 1:关闭 2:低功耗 3:低噪声 */
			ICM42605_Write_Byte(ICM42605_PWR_MGMT0, reg_val);
			delay_ms(1); 		/* 操作完PWR—MGMT0寄存器后 200us内不能有任何读写寄存器的操作 */
			return 0;
	}
		else return 1;
}

/**-------------------------------------------------------------------------------------------------------------------
  * @brief    均值滤波获得温度值
  * @param    *ICM42605_Temperature ：实际温度值
  * @return   温度值
  * @note     使用模拟FIFO求10次采样的平均值
  * @example  MPU_Get_Temperature(&MPU6050_Temperature); 变量MPU6050_Temperature即为测得的温度。
  * @date     2022/6/21 16:15
  * @Author   WxxGreat
  *-----------------------------------------------------------------------------------------------------------------*/
#ifndef Temperature_FIFO_SIZE  
#define Temperature_FIFO_SIZE 10  //采样数据个数
#endif
static short ICM42605_Temperature_FIFO[Temperature_FIFO_SIZE] = {0};
void Get_ICM42605_Tempdata(float* ICM42605_Temperature)
{
  static unsigned char ICM42605_Temperature_Index;

	uint8_t buf[2];
	uint8_t i ;
	
	ICM42605_Read_Len(ICM42605_ADDRESS, ICM42605_TEMP_DATA1, 2, buf);

	ICM42605_Temperature_FIFO[ICM42605_Temperature_Index] =  ((unsigned short int)buf[0] << 8) | buf[1];//将读到的温度值放到FIFO队列中
	ICM42605_Temperature_Index = (ICM42605_Temperature_Index + 1) % Temperature_FIFO_SIZE;//更新下一个值进入FIFO队列中的位置
	
	for (i = 0; i < Temperature_FIFO_SIZE; i++)  *ICM42605_Temperature += ICM42605_Temperature_FIFO[i];//对FIFO中所有的值累加

	*ICM42605_Temperature =   (*ICM42605_Temperature / Temperature_FIFO_SIZE / 132.48f) + 25;
}




//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取ICM42605加速度计数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void Get_ICM42605_accdata(short *ax, short *ay, short *az)
{
    uint8_t dat[6];

	  ICM42605_Read_Len(ICM42605_ADDRESS, ICM42605_ACCEL_DATA_X1, 6, dat);
	
    *ax = (int16_t)(((int16_t)dat[0]<<8 | dat[1]));
    *ay = (int16_t)(((int16_t)dat[2]<<8 | dat[3]));
    *az = (int16_t)(((int16_t)dat[4]<<8 | dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取ICM42605陀螺仪数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void Get_ICM42605_gyro(short *gx, short *gy, short *gz)
{
    uint8_t dat[6];

	 ICM42605_Read_Len(ICM42605_ADDRESS, ICM42605_GYRO_DATA_X1, 6, dat);
	
    *gx = (int16_t)(((int16_t)dat[0]<<8 | dat[1]));
    *gy = (int16_t)(((int16_t)dat[2]<<8 | dat[3]));
    *gz = (int16_t)(((int16_t)dat[4]<<8 | dat[5]));
}



//IIC连续写
//addr:器件地址
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//其他,错误代码
uint8_t ICM42605_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
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
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//其他,错误代码
uint8_t ICM42605_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
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
//IIC写一个字节
//reg:寄存器地址
//data:数据
//返回值:0,正常
//其他,错误代码
uint8_t ICM42605_Write_Byte(uint8_t reg, uint8_t data)
{
	IIC_Start();
	IIC_Send_Byte((ICM42605_ADDRESS << 1) | 0); //发送器件地址+写命令
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
//IIC读一个字节
//reg:寄存器地址
//返回值:读到的数据
uint8_t ICM42605_Read_Byte(uint8_t reg)
{
	uint8_t res;
	IIC_Start();
	IIC_Send_Byte((ICM42605_ADDRESS << 1) | 0); //发送器件地址+写命令
	IIC_Wait_Ack();						//等待应答
	IIC_Send_Byte(reg);					//写寄存器地址
	IIC_Wait_Ack();						//等待应答
	IIC_Start();
	IIC_Send_Byte((ICM42605_ADDRESS << 1) | 1); //发送器件地址+读命令
	IIC_Wait_Ack();						//等待应答
	res = IIC_Read_Byte(0);				//读取数据,发送nACK
	IIC_Stop();							//产生一个停止条件
	return res;
}

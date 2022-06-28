#ifndef _IMU_H
#define _IMU_H

#include "Mahony.h"
#include "ICM20602orMPU6050.h"
#include "ICM42605.h"
#include "i2c.h"
#include "delay.h"
#include <stdio.h>
#include <math.h>

extern short aacx,aacy,aacz;		//加速度传感器原始数据  angular acceleration
extern short gyrox,gyroy,gyroz;	//陀螺仪原始数据  gyroscope
extern float Roll,Pitch,Yaw;

typedef struct
{
	float Pitch;
	float Roll;
	float Yaw;

	float Pitch_v;
	float Roll_v;
	float Yaw_v;

	float ax;
	float ay;
	float az;

} IMU_Info;
#include "Mahony.h"
#include "ICM20602orMPU6050.h"
#include "ICM42605.h"
#include "i2c.h"
#include "delay.h"
#include <stdio.h>
#include <math.h>
extern IMU_Info imu;
extern float IMU_Temperature;


void IMU_Init(void);
void IMU_Update(void);
void IMU_readGyro_Acc(signed short int *gyro, signed short int *acc);
void IMU_Init_Offset(void);

#endif

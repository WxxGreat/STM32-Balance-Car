# 0. 关于本项目

本项目使用`STM32F103C8T6`作为主控，`Keil5`开发，`Mahony`算法进行姿态解算的平衡车。项目中给出了`MPU6050`、`ICM20602`、`ICM42605`三种主流IMU的驱动。目前仅实现了直立平衡，在设计设加入了BlueTooth模块，手机遥控部分还在开发中（新建文件夹）

 (＠_＠;)



# 1. 硬件

## 1.1 PCB 

目前是第一版，用到的都是插接件。后续还会更新PCB（计划换成贴片，体积会更小）

<img src="README.assets\0F15BE167BFB33291626298335F3527D.png"/>

<img src="README.assets\YXY7YZQDE56HT2FE4IG3I.jpg"/>



## 1.2 车模

<img src="README.assets\41622DE8C57076C94199A25C662CC42F.jpg">

`LCD`使用1.3寸`ST7789`主控 `240×240`分辨率



电机型号`MG513P20_12V` ，我买的淘宝店：[直流减速电机带光电霍尔编码器码盘测速两轮自平衡小车 ](https://item.taobao.com/item.htm?spm=a1z10.3-c-s.w4002-15726392041.27.16a3143eqzsQfR&id=45347924687 )*

电池链接：[恩智浦智能汽车竞赛 正品BCDEFHLM车模2S锂电池及充电器](https://item.taobao.com/item.htm?spm=a1z10.3-c-s.w4002-22508770840.25.3b1949cceHXuh9&id=610152052172 )*

SolidWorks画的简单底板尺寸如图，用3D打印机制造，源文件在学校电脑上，回学校了再上传



<img src="README.assets\123456789.png"/>





# 2. 算法

## 2.1 `PID`算法

####   2.1.1理论分析

该项目为使的小车平衡最主要的两个环就是直立环和速度环，通过叠加得到最终给电机的输出，这也是网上绝大多数的小车平衡控制方法。

1. 我们先使用常规的速度负反馈算法想象一下。首先我们给定一个目标速度值，由于小车在直立控制的作用下，此时小车要向前倾斜以获取加速度，车轮需要往后运动，这样小车速度就会下降。因为是负反馈,速度下降之后，速度控制的偏差增大，小车往前倾斜的角度增大，如此反复，小车便会倒下。
2. 为保证直立控制的优先级，我们把速度控制放在直立控制的前面，也就是速度控制调节的结果仅仅是改变直立控制的目标值。因为根据经验可知，小车的运行速度和小车的倾角是相关的。比如要提高小车向前行驶的速度，就需要增加小车向前倾斜的角度，倾斜角度加大之后，车轮在直立控制的作用下需要向前运动。因此直立环的`Kp`后面乘的值并不是`当前角度-机械中值`，而是一个我们速度的期望。**（这个理解了很重要！！！！）**

$$
a_1=Kp*(\theta-a_2)+Kd*\theta'  \qquad···································①直立环
$$

$$
a_2=Kp_1* Err_{speed}+Ki_1 * \sum Err_{speed}\space   \qquad····························②速度环
$$


**合并一下就可以得到：**



$$
a=Kp*(\theta)+Kd*\theta'-Kp \left [kp_1 * Err_{speed}+ki_1 * \sum Err_{speed} \right ] \qquad········③最终式
$$



因此在代码实现上我们就可以实现两个环的直接相加或相减，在`TIM3`定时器中`10ms`一个周期进行控制。

```c
      Balance_PID_Result = Position_PID_Cal(&Balance_PID, imu.Roll + 0.3f);
      if (Time_GAP_20ms) Velocity_PID_Result = Position_PID_Cal(&Velocity_PID, Velocity);

      PID_Result = Balance_PID_Result + Velocity_PID_Result;
      Set_Motor_Speed(PID_Result, PID_Result);
```

####   2.1.2调参经验

根据上边的分析我们只需要分别调整直立环的`Kp，kd`和速度环的`Kp，Ki`

1. 对于直立环的`Kp`，是调整最方便观察现象的，太小时小车没有足够的恢复力，太大时小车会在中值附近大幅震荡，调整到一个略微震荡的值即可
2. 直立环的`Kd`作用是减小低频振荡，但`Kd`太大小车又会造成高频振动，从小到大增大`Kd`，直到小车出现小幅高频振荡
3. 直立环调整结束后小车可以平衡，但受到扰动便会往一个方向疯跑，现在引入直立环
4. 网上查到的资料对于速度环`Ki = Kp/200` ，但在我的实际调整中最终确定了`Ki = Kp/100`，这个看自己小车了，可以先按照`Ki = Kp/200`去调
5. 先将直立环`Kp，Kd`同时×0.8。调整速度环`Kp`，速度环`Kp`越大，小车便越不会出现向一个方向狂奔的情况（因为速度被速度环控住了），但会减弱直立环的控制效果，因此调整到一个车受到干扰会摇摇晃晃停下的一个状态。
6. 摇摇晃晃的原因就是因为速度环`Kp`太大，回调速度环`Kp`，并且增大直立环`Kd`（想想`Kd`的作用是什么呢？）

**以上就是我调参的经验，可以参考，最好可以理解原理再去上手实践。**

## 2.2 `Mahony`算法

**参考一篇文章：[基于Manony滤波算法的姿态解算](https://blog.csdn.net/weixin_44821644/article/details/108542178)**

###   2.2.1IMU（以MPU6050举例）

`MPU6050`是一个集成了陀螺仪和加速度计的传感器，它能输出在直角坐标系下的`x，y，z`轴的角速度和加速度数据。

陀螺仪输出的格式为：绕`x`轴的旋转角速度，绕`y`轴的角速度，绕`z`轴的角速度（分别称为`roll`角速度，pitch角速度和`yaw`角速度）。

加速度计输出的格式为：`x`轴的加速度，`y`轴的加速度，`z`轴的加速度。

另外还需要关注传感器的其他参数如：

- 陀螺仪的量程：`eg.±2000dps`
- 加速度计的量程：`eg.±2g`
- `ADC`转换精度为`16bit`
- 传感器采样率`4-1000hz：eg.1000hz`


我们从IMU那就得到了陀螺仪数据`gx,gy,gz`，加速度数据`az,ay,az`

螺仪转换精度**2^16=65536   ,  65536/{2000-(-2000)}=16.4,实际1°等于ADC值16.4**

采样率就是数据的更新率，也就是我们每次读取数据的频率。

首先将陀螺仪的数据转换成角度，这里封装成一个函数

###   2.2.2算法实现

```c
static void Get_IMU_Values(float *values)
{
	int16_t gyro[3],acc[3];
	IMU_readGyro_Acc(&gyro[0],&acc[0]);
	for(int i=0;i<3;i++)
	{
        //gyro range +-2000; adc accuracy 2^16=65536; 65536/4000=16.4;
		values[i]=((float) gyro[i])/16.4f;	
		values[3+i]=(float) acc[i];
	}
}
```

**然后编写函数实现计算姿态角的功能，使用四元数计算姿态角的公式在理论分析中推导：**

**其中α为绕x轴旋转角即`roll`，β为绕y轴旋转角即`pitch`，γ为绕z轴旋转角即yaw。`a,b,c,d`即`q0,q1,q2,q3`.**


```c
void IMU_Update(void)
{
	static float q[4];
	float Values[6];	
	Get_IMU_Values(Values);	
	
	//change angle to radian,the calculate the imu with Mahony
	MahonyAHRSupdateIMU(Values[0] * PI/180, Values[1] * PI/180, Values[2] * PI/180,
 	                    Values[3], Values[4], Values[5]);		
	//save Quaternion
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

	//calculate the imu angle with quaternion
	imu.Roll = (atan2(2.0f*(q[0]*q[1] + q[2]*q[3]),1 - 2.0f*(q[1]*q[1] + q[2]*q[2])))* 180/PI;	
	imu.Pitch = -safe_asin(2.0f*(q[0]*q[2] - q[1]*q[3]))* 180/PI;
	imu.Yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 180/PI;
}
```

**代码中`MahonyAHRSupdateIMU()`函数实现的就是四元数的更新算法。**

*逻辑上，首先用加速度计校准陀螺仪，方式是通过计算当前四元数姿态下的重力分量，与加速度计的重力分量作叉积，得到误差。
对误差作P(比例)和I(积分)运算后加到陀螺仪角速度上。最终由角速度计算新的四元数。*



**代码中的 `sampleFreq` 即执行姿态解算的频率，这里用定时器，以500HZ的频率调用`get_angle();`**

```c
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;	
	float halfvx, halfvy, halfvz;	//1/2 重力分量
	float halfex, halfey, halfez;	//1/2 重力误差
	float qa, qb, qc;
	//对加速度数据归一化
	recipNorm = invSqrt(ax * ax + ay * ay + az * az);
	ax *= recipNorm;
	ay *= recipNorm;
	az *= recipNorm;

	// 由四元数计算重力分量
	halfvx = q1 * q3 - q0 * q2;
	halfvy = q0 * q1 + q2 * q3;
	halfvz = q0 * q0 - 0.5f + q3 * q3;

	// 将四元数重力分量 与 加速度计重力分量 作叉积 得到误差
	halfex = (ay * halfvz - az * halfvy);
	halfey = (az * halfvx - ax * halfvz);
	halfez = (ax * halfvy - ay * halfvx);

	//对误差作积分
	integralFBx += twoKi * halfex * (1.0f / sampleFreq);	
	integralFBy += twoKi * halfey * (1.0f / sampleFreq);
	integralFBz += twoKi * halfez * (1.0f / sampleFreq);
	//反馈到角速度
	gx += integralFBx;	     gy += integralFBy;       gz += integralFBz;

	// 对误差作比例运算并反馈
	gx += twoKp * halfex;    gy += twoKp * halfey;    gz += twoKp * halfez;

	// 计算1/2 dt
	gx *= (0.5f * (1.0f / sampleFreq));		
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;     qb = q1;      qc = q2;
	// 更新四元数
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// 四元数归一化
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;     q1 *= recipNorm;     q2 *= recipNorm;     q3 *= recipNorm;
}
```

**由于加速度计对水平方向的旋转无能为力，故用此程序得到的`yaw`角数据会一直漂移，无法得到校准；通常的解决方法是增加一个磁场传感器，来获得一个准确的水平方向角来校准陀螺仪的漂移。`MPU6050`支持扩展一个`IIC`接口到磁场传感器，可通过配置`MPU6050`的`IIC MASTER` 来读取磁场传感器的数据。**

在`Mahony`中提供了包含磁场数据的融合函数：

> ```C
> void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, 
>                          float mx, float my, float mz);
> ```



## 3. 程序逻辑

首先看main函数：

```c
int main(void)
{
	All_HardWare_init();
	while (1)
	{
		Protect_Check();
		LED_show_working();
		LCD_show_Brief_info();
	}
}
```

代码都封装在了函数里，因此主控的main函数非常简单。`All_HardWare_init();`包含了所有硬件、片内资源的初始化。**因为`Mahony`每次上电融合解算姿态时需要几秒的自我校准，因此先打开定时器。**

```c
	TIM3_Int_Init(99, 7199); // 72M ÷7200 ÷100 = 10 ms
```

几秒过后再初始化PID控制器

```c
	PID_init();              //直立环，速度环PID控制器初始化
```

**`定时器3中断服务函数`在`control.c`文件中，包含姿态解算和PID控制。**

```c
void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) 
  {   TIM_ClearITPendingBit(TIM3, TIM_IT_Update);             
     /**
      *PID控制与姿态解算部分，详情请TP至
      *....../主控程序/HARDWARE/control/control.c
      *
      */
  }
}
```



死循环中目前三个函数分别是LED、LCD状态显示和 一个简易的过倾保护。

```c
void Protect_Check(void)
{
	if (imu.Roll > 30 || imu.Roll < -30)
		Protect = 1;
	else
		Protect = 0;
}
```




## 4. 补充

> ```
> NULL
> ```

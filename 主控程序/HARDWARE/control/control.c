#include "control.h"


PID Balance_PID , Velocity_PID , Turn_PID;
u8  Protect = 1;
float Velocity,LandR_Motor_Bias;
s16 Balance_PID_Result;
s16 Velocity_PID_Result;
s16 Turn_PID_Result;
s16 PID_Result;
u32 TIM3_Tick;


/**
  * @brief    TIM3中断服务函数
  *
  * @note     车身平衡控制
  *
  * @date     2022/6/6 13:54
  *
  * @author   WxxGreat
  * 
  * @version  1.0 仅能平衡 
  **/
void TIM3_IRQHandler(void){ 
if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) //检查TIM3更新中断发生与否
{   TIM_ClearITPendingBit(TIM3, TIM_IT_Update );  //清除TIM3更新中断标志  
	TIM3_Tick++;
//————————————————————————————————————————————————————————//		
	IMU_Update();
	
 // printf("%.2f\r\n",imu.Roll);
	if (Protect)  SYSTEM_STOP;
	else  Balance_Control(0);	
	
//————————————————————————————————————————————————————————//		
}}






/**-------------------------------------------------------------------------------------------------------------------
  * @brief    PID初始化函数
  *
  * @note     NULL
  *
  * @return   NULL
  *
  * @date     2022/6/27 15:48
  *
  * @author   WxxGreat
  *-----------------------------------------------------------------------------------------------------------------*/
void PID_init(void)
{
#if USE_ICM42605 == 1
	Balance_PID.Kp = 270;	//-400
	Balance_PID.Kd = 1300; //-2000

	Velocity_PID.Kp = 100;
	Velocity_PID.Ki = Velocity_PID.Kp / 100;
	Velocity_PID.SetPoint = 0; 
#else
  Balance_PID.Kp = -200;	//-265
	Balance_PID.Kd = -1480; //-1750
	Balance_PID.SetPoint = 0;

	Velocity_PID.Kp = -82.5;//-135
	Velocity_PID.Ki = Velocity_PID.Kp / 200;
	Velocity_PID.SetPoint = 0;
	
	Turn_PID.Kp = 20;
#endif
	
	PID_Init(&Balance_PID, Balance_PID.SetPoint, Balance_PID.Kp, 0, Balance_PID.Kd);		 //初始化直立环PID
	PID_Init(&Velocity_PID, Velocity_PID.SetPoint, Velocity_PID.Kp, Velocity_PID.Ki, 0); //初始化速度环PID
	PID_Init(&Turn_PID, 0, Turn_PID.Kp, 0, 0);                                           //初始化速度环PID
}



void Balance_Control(unsigned char Target_Velocity)
{
	s16 Left , Right , a;
	
	Get_now_Velocity(&Velocity , &LandR_Motor_Bias );

	Balance_PID_Result = Position_PID_Cal(&Balance_PID, imu.Roll + 0.3f);
	
//	Velocity_PID_Result = Position_PID_Cal(&Velocity_PID, Velocity);
	
	Velocity_PID_Result = Balance_Car_Velocity_ONLY_Position_PID_Cal(&Velocity_PID, Velocity , Target_Velocity);

	Turn_PID_Result = Position_PID_Cal(&Turn_PID, LandR_Motor_Bias);
	
	a = Balance_PID_Result + Velocity_PID_Result ;
	
	Left = a + Turn_PID_Result;
	Right = a - Turn_PID_Result;
	
	Set_Motor_Speed(Left, Right);
}


/**-------------------------------------------------------------------------------------------------------------------
  * @brief    倾角过大关闭电机
  *
  * @note     变量Protect作用于TIM3_IRQHandler(void)
  *
  * @param    Protect == 1 时表明需要触发保护
  *
  * @date     2022/6/27 16:20
  *
  * @author   WxxGreat
  *-----------------------------------------------------------------------------------------------------------------*/
void Protect_Check(void)
{
	if (imu.Roll > 30 || imu.Roll < -30)
	{
		Protect = 1;
		Velocity_PID.SumError = 0;
	}
	else
		Protect = 0;
}


/**-------------------------------------------------------------------------------------------------------------------
  * @brief    展示程序正常运行
  *
  * @note     因为while(1)中每次运行的间隔不固定，因此用<=50
  *
  * @return   NULL
  *
  * @date     2022/6/27 21:45
  *
  * @author   WxxGreat
  *-----------------------------------------------------------------------------------------------------------------*/
void LED_show_working(void)
{
	if (TIM3_Tick % 100 <= 20)  LED0 = ~LED0; 
}


/**-------------------------------------------------------------------------------------------------------------------
  * @brief    获取δt时间内编码器度数，δt足够小，可看成小车速度
  *
  * @note     
  *
  * @return   NULL
  *
  * @date     2022/7/1 17:15
  *
  * @author   WxxGreat
  *-----------------------------------------------------------------------------------------------------------------*/
void Get_now_Velocity(float *Car_Velocity , float *LandR_Motor_Bias )
{
	s16 a,b;
	
  a= TIM2 -> CNT;
	b= TIM4 -> CNT; 
	TIM2 -> CNT=0;
	TIM4 -> CNT=0;
	*Car_Velocity = -( a + b );
	*LandR_Motor_Bias += a - b ;
}



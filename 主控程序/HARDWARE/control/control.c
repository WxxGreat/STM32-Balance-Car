#include "control.h"


PID Balance_PID , Velocity_PID;
u8  Protect = 1;
s16 Velocity;
s16 Balance_PID_Result;
s16 Velocity_PID_Result;
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
{ 
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update );  //清除TIM3更新中断标志  
	TIM3_Tick++;
//————————————————————————————————————————————————————————//		
	IMU_Update();

	if (Protect)  System_STOP;
	else
	{
		Velocity = (Read_Encoder(2) + Read_Encoder(4)) / 2;
		Balance_PID_Result = Position_PID_Cal(&Balance_PID, imu.Roll + 0.3f);

		if (Time_GAP_20ms) Velocity_PID_Result = Position_PID_Cal(&Velocity_PID, Velocity);
		PID_Result = Balance_PID_Result + Velocity_PID_Result ;

		Set_Motor_Speed(PID_Result, PID_Result);
	}	
		
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

	Velocity_PID.Kp = 135;
	Velocity_PID.Ki = Velocity_PID.Kp / 100;
	Velocity_PID.SetPoint = 0; 
#else
  Balance_PID.Kp = -222;	//-270
	Balance_PID.Kd = -1320; //-1500
	Balance_PID.SetPoint = 0;

	Velocity_PID.Kp = -150;//-135
	Velocity_PID.Ki = Velocity_PID.Kp / 95;
	Velocity_PID.SetPoint = 0;
#endif
	
	PID_Init(&Balance_PID, Balance_PID.SetPoint, Balance_PID.Kp, 0, Balance_PID.Kd);		 //初始化直立环PID
	PID_Init(&Velocity_PID, Velocity_PID.SetPoint, Velocity_PID.Kp, Velocity_PID.Ki, 0); //初始化速度环PID
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
		Protect = 1;
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
	if (TIM3_Tick % 100 <= 50)  LED0 = ~LED0; 
}


#include "PID.h"

/********************************增量式PID计算********************************
增量式PID速度环控制
1st：调整Ki，求起振点，过大会导致过冲
2nd：调节Kp，抑制振荡，过大会导致大幅度的震荡
3rd：调节Kd，削弱超调，过大会导致低频振荡
******************************************************************************/
float Increment_PID_Cal(PID *s_PID, float now_point)
{
  s_PID->LastResult = s_PID->Result; //将上次该函数计算的Result的值放入LastResult

  s_PID->Error = s_PID->SetPoint - now_point; //误差计算

  // PID计算
  s_PID->Result =   s_PID->LastResult                                                       //上次结果
                  + s_PID->Kp * (s_PID->Error - s_PID->LastError)                           // 比例项
                  + s_PID->Ki * s_PID->Error                                                // 积分项
                  + s_PID->Kd * (s_PID->Error - 2 * (s_PID->LastError) + s_PID->PrevError); // 微分项

  s_PID->PrevError = s_PID->LastError; // 将上次误差值(LastError)放入前一次误差(PrevError)中
  s_PID->LastError = s_PID->Error;     // 将本次函数计算的误差(Error)放入上次误差中(LastError)

  PID_Output_limit(s_PID, &s_PID->Result); //输出限幅

  return s_PID->Result;
}



/*******************************位置式PID计算************************************/
float Position_PID_Cal(PID *s_PID, float now_point)
{
	float IOutValue ;
  s_PID->LastResult = s_PID->Result; //将上次该函数计算的Result的值放入LastResult

  s_PID->Error = s_PID->SetPoint - now_point; //误差计算
	
	//s_PID->Error = s_PID->Error * 0.9f + s_PID->LastError * 0.2f;	//一阶低通滤波

  s_PID->SumError += s_PID->Error; //积分误差累加
	
	
	
  IOutValue = s_PID->SumError * s_PID->Ki; //积分计算
  PID_Integral_limit(s_PID, &IOutValue); //积分限幅

  // PID计算
  s_PID->Result = s_PID->Kp * s_PID->Error                         // 比例项
                  + IOutValue                                      // SumError * Ki
                  + s_PID->Kd * (s_PID->Error - s_PID->LastError); // 微分项

  s_PID->PrevError = s_PID->LastError; // 将上次误差值(LastError)放入前一次误差(PrevError)中
  s_PID->LastError = s_PID->Error;     // 将本次函数计算的误差(Error)放入上次误差中(LastError)

  PID_Output_limit(s_PID, &s_PID->Result); //输出限幅

  return s_PID->Result;
}



/**-------------------------------------------------------------------------------------------------------------------
  * @brief    平衡车速度环专属PID
  *
  * @note     平衡车速度环专属PID  !!!!!!!!!
  *
  * @param    State_Flag  
  *           0: 直立平衡状态
  *           1: 前进模式
  *          -1: 后退模式
  *
  * @date     2022/7/2 22:38
  *
  * @author   WxxGreat
  *-----------------------------------------------------------------------------------------------------------------*/
float Balance_Car_Velocity_ONLY_Position_PID_Cal(PID *s_PID, float now_point , unsigned char State_Flag)
{
	float IOutValue ;
  s_PID->LastResult = s_PID->Result; //将上次该函数计算的Result的值放入LastResult

  s_PID->Error = s_PID->SetPoint - now_point; //误差计算
	
//	s_PID->Error = s_PID->Error * 0.8f + s_PID->LastError * 0.2f;	//一阶低通滤波
	
  s_PID->SumError += (s_PID->Error + State_Flag * 90 ); //积分误差累加
	
  IOutValue = s_PID->SumError * s_PID->Ki; //积分计算
  PID_Integral_limit(s_PID, &IOutValue); //积分限幅

  // PID计算
  s_PID->Result = s_PID->Kp * s_PID->Error                         // 比例项
                  + IOutValue                                      // SumError * Ki
                  + s_PID->Kd * (s_PID->Error - s_PID->LastError); // 微分项

  s_PID->PrevError = s_PID->LastError; // 将上次误差值(LastError)放入前一次误差(PrevError)中
  s_PID->LastError = s_PID->Error;     // 将本次函数计算的误差(Error)放入上次误差中(LastError)

  PID_Output_limit(s_PID, &s_PID->Result); //输出限幅

  return s_PID->Result;
}


/*****************************比例外置式PID计算**********************************/
float PID_Cal(PID *s_PID, float now_point) 
{
	float IOutValue;
  s_PID->LastResult = s_PID->Result; // 简单赋值运算

  s_PID->Error = s_PID->SetPoint - now_point; //误差计算

  s_PID->SumError += s_PID->Error; //积分误差累加

  IOutValue = s_PID->SumError * s_PID->Ki; //积分项计算

  PID_Integral_limit(s_PID, &IOutValue); //积分限幅

  // PID计算
  s_PID->Result = s_PID->Kp * (s_PID->Error + IOutValue + s_PID->Kd * (s_PID->Error - s_PID->LastError));

  s_PID->PrevError = s_PID->LastError; // 简单赋值运算
  s_PID->LastError = s_PID->Error;     // 简单赋值运算
  //输出限幅
  PID_Output_limit(s_PID, &s_PID->Result); //输出限幅

  return s_PID->Result;
}



void PID_Init(PID *s_PID, float target, float PID_Kp, float PID_Ki, float PID_Kd) //初始化PID结构体参数
{
  s_PID->SetPoint = target;
  s_PID->Kp = PID_Kp;
  s_PID->Ki = PID_Ki;
  s_PID->Kd = PID_Kd;
  s_PID->Error = 0;
  s_PID->LastError = 0;
  s_PID->PrevError = 0;
  s_PID->SumError = 0;
  s_PID->LastResult = 0;
  s_PID->Result = 0;
  s_PID->OutMax = DEFAULT_PID_OUT_MAX;
  s_PID->OutMin = DEFAULT_PID_OUT_MIN;
  s_PID->IntegralMax = DEFAULT_PID_INTEGRAL_OUT_MAX;
  s_PID->IntegralMin = DEFAULT_PID_INTEGRAL_OUT_MIN;
}

void PID_SetPoint(PID *s_PID, float target) //设置目标值
{
  s_PID->SetPoint = target;
}

void PID_Set_out_Range(PID *s_PID, float outMax, float outMin) //设置PID输出范围
{
  s_PID->OutMax = outMax;
  s_PID->OutMin = outMin;
}

void PID_Set_Integral_out_Range(PID *s_PID, float outMax, float outMin) //设置PID积分范围
{
  s_PID->IntegralMax = outMax;
  s_PID->IntegralMin = outMin;
}

void PID_Integral_limit(PID *s_PID, float *IOutValue) //积分限幅
{
  if (*IOutValue > s_PID->IntegralMax)
  {
    *IOutValue = s_PID->IntegralMax;
  }
  else if (*IOutValue < s_PID->IntegralMin)
  {
    *IOutValue = s_PID->IntegralMin;
  }
}

void PID_Output_limit(PID *s_PID, float *Result) //输出限幅
{
  if (*Result > s_PID->OutMax)
  {
    *Result = s_PID->OutMax;
  }
  else if (*Result < s_PID->OutMin)
  {
    *Result = s_PID->OutMin;
  }
}


float Yaw_PID(PID *s_PID, float now_point) //
{
  // PID计算
  s_PID->Result = s_PID->Kp * now_point +  s_PID->Kd * (now_point - s_PID->LastResult);
	
  s_PID->LastResult = now_point; // 简单赋值运算
	
  PID_Output_limit(s_PID, &s_PID->Result); //输出限幅

  return s_PID->Result;
}


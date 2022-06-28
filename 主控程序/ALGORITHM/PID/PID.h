#ifndef __PID_H
#define __PID_H

//PID输出范围宏定义
#define DEFAULT_PID_OUT_MAX ( 7100)
#define DEFAULT_PID_OUT_MIN (-7100)

//PID积分范围宏定义
#define DEFAULT_PID_INTEGRAL_OUT_MAX (7000)
#define DEFAULT_PID_INTEGRAL_OUT_MIN (-7000)


//定义结构体
typedef struct
{
    float SetPoint;    // 设定目标
    float Kp;          // 比例常数
    float Ki;          // 积分常数
    float Kd;          // 微分常数
    float SumError;    // 积分和
    float Error;       // 误差
    float LastError;   // 上次误差
    float PrevError;   // 前一次误差
    float LastResult;  // 上次计算结果
    float Result;      // 当前计算结果
    float OutMax;      // 输出限幅最大值
    float OutMin;      // 输出限幅最小值
    float IntegralMax; // 积分限幅最大值
    float IntegralMin; // 积分限幅最小值
} PID;


void PID_Init(PID *s_PID, float set_point, float Kp, float Ki, float Kd); //PID初始化

void PID_SetOutRange(PID *s_PID, float outMax, float outMin);         //设置PID输出范围
void PID_SetIntegralOutRange(PID *s_PID, float outMax, float outMin); //设置PID积分范围
void PID_SetPoint(PID *s_PID, float set_point);                              //设置目标值

float Increment_PID_Cal(PID *s_PID, float now_point); //增量式PID计算
float Position_PID_Cal(PID *s_PID, float now_point);  //位置式PID计算
float PID_Cal(PID *s_PID, float now_point);           //比例外置式PID

void PID_Integral_limit(PID *s_PID, float *IOutValue);
void PID_Output_limit(PID *s_PID, float *Result);
float Yaw_PID(PID *s_PID, float now_point); //


#endif

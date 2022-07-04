#ifndef __CONTROL_H
#define __CONTROL_H

#include "sys.h"
#include "PID.h"
#define SYSTEM_STOP   Set_Motor_Speed(0, 0) 


void PID_init(void);
void Protect_Check(void);
void LED_show_working(void);
void Get_now_Velocity(float *Car_Velocity , float *LandR_Motor_Bias );
void Balance_Control(unsigned char Target_Velocity);



extern PID	Balance_PID;
extern PID	Velocity_PID;
extern u8 Protect;
extern unsigned int TIM3_Tick;

#endif


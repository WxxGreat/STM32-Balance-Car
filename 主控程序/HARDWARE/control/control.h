#ifndef __CONTROL_H
#define __CONTROL_H

#include "sys.h"
#include "PID.h"
#define System_STOP   Set_Motor_Speed(0, 0) 
#define Time_GAP_20ms  TIM3_Tick % 20 == 0

void PID_init(void);
void Protect_Check(void);
void LED_show_working(void);





extern PID	Balance_PID;
extern PID	Velocity_PID;
extern u8 Protect;
extern unsigned int TIM3_Tick;

#endif


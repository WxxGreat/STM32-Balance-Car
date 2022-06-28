#ifndef __MOTOR_H
#define __MOTOR_H
#include "sys.h"


void TIM1_Motor_PWM_Init(unsigned short int arr,unsigned short int psc); 
void Motor_Dir_Init(void);
void MotorDriver_Init(void);
void Set_Motor_Speed(signed short int Left,signed short int Right);
#define PWMA   TIM1->CCR4    //B1
#define AIN2   PBout(14)     //
#define AIN1   PBout(15)     //

#define PWMB   TIM1->CCR1    //B0
#define BIN1   PBout(12)     // 
#define BIN2   PBout(13)     // 
#endif

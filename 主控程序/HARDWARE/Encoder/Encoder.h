#ifndef _ENCODER_H
#define _ENCODER_H

#include "sys.h"

void Encoder_Init_TIM2(void);
void Encoder_Init_TIM4(void);
int Read_Encoder(u8 TIMX);


#endif


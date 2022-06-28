#include "LED.h"


//LED IO初始化
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;        //LED0-->PC.13 端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;  //IO口速度为2MHz
    GPIO_Init(GPIOC, &GPIO_InitStructure);            //根据设定参数初始化GPIOC.13
    GPIO_SetBits(GPIOC, GPIO_Pin_13);                 //PC.13 输出高
}

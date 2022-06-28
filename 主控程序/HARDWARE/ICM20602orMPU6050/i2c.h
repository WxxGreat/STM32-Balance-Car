#ifndef __IOI2C_H
#define __IOI2C_H
#include "sys.h"

/****************SDA和SCL口配置**************************/
#define IIC_SDA_GPIO_Pin           GPIO_Pin_10
#define IIC_SCL_GPIO_Pin           GPIO_Pin_11
#define IIC_GPIOx                  GPIOB
#define IIC_RCC_APB2Periph_GPIOx   RCC_APB2Periph_GPIOB

#define IIC_SCL                    PBout(11) //SCL
#define IIC_SDA                    PBout(10) //SDA	 
#define READ_SDA                   PBin (10) //输入SDA 
/********************************************************/



/****************以下三个函数F103 →  F407 在GPIO配置有改变****************************/
void SDA_IN(void);  //IO方向设置
void SDA_OUT(void);
void IIC_Init(void);        //初始化IIC的IO口		
/*************************************************************************************/




//IIC所有操作函数
	 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(uint8_t txd);			//IIC发送一个字节
uint8_t IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
uint8_t IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	  


#endif

//------------------End of File----------------------------

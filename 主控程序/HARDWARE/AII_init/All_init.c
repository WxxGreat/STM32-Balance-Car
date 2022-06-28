#include "All_init.h"

void All_RCC_clock_init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE); //使能ABC端口时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);												//使能定时器1的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);												//使能USART1时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);	//使能定时器234的时钟
}

void All_HardWare_init(void)
{
	All_RCC_clock_init();

	JTAG_Set(JTAG_SWD_DISABLE); //关闭JTAG接口
	JTAG_Set(SWD_ENABLE);		    //打开SWD接口 可以利用主板的SWD接口调试
	delay_init();				        //延时函数初始化

	LED_Init();
	LCD_Init();
	MotorDriver_Init();
	uart_init(115200);

	Encoder_Init_TIM2(); // right
	Encoder_Init_TIM4(); // left

  Set_Motor_Speed( 0 , 0 );
	delay_ms(1000);
	/****************IMU初始化*******************/
#if USE_MPU6050DMP == 1
	IMU_Init();			   // 陀螺仪初始化
	while (mpu_dmp_init()) // 陀螺仪-dmp初始化
	{
		LCD_printf(0, 0, "mpu dmp error:%d", mpu_dmp_init());
		delay_ms(1);
	}
#else
	IMU_Init(); // 陀螺仪初始化
#endif
	/*******************************************/


	TIM3_Int_Init(99, 7199); // 72M ÷7200 ÷100 = 10 ms

//	while (TIM3_Tick < 500)
//	{
//		LCD_printf(200, 90, "%d", TIM3_Tick / 10 * 2);
//		Protect_Check();
//	}
//	LCD_ShowChinese(0, 90, "陀螺仪校准成功", WHITE, BLACK, 32, 0);
//	while (TIM3_Tick < 650)
//	{
//		LCD_ShowChar(224, 90, '.', WHITE, BLACK, 32, 0);
//		Protect_Check();
//	}

	LCD_Fill(0, 90, LCD_W, 122, BLACK);
	PID_init();
}

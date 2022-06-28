#include "sys.h"

/**
 * @brief          使用Mahony进行姿态解算的平衡车
 *
 * @note           Forkde from my former project "MPU6050 DMP Demo For STM32F10x" 
 *
 * @date           2021-8-20 16:32
 *
 * @LastEditTime   2022-6-28 16:05
 *
 * @author         WxxGreat
 *
 * @version        1.0 仅可以平衡  
 */

int main(void)
{
	All_HardWare_init();

	while (1)
	{
		Protect_Check();
		LED_show_working();
		LCD_show_Brief_info();
	}
	
	
}

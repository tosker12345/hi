#include "stm32f3xx_hal.h"

void f103_72m_delay_us(uint32_t us) //STM32F103,72MHz, delay 1us 
{
	if(us>1)
	{
		volatile uint32_t count = us*8-4;
		while(count--); 
	}
	else
	{
		volatile uint32_t count = 4;
		while(count--); 
	}
}

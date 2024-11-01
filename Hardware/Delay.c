#include "stm32f4xx.h"                  // Device header
void Delay_us(int32_t time)
{
	int32_t i;
	int32_t time1=time*30;
	for(i=time1;i>=0;i--)
	{
		
	}
}

void Delay_ms(int32_t time)
{
	int32_t i;
	int32_t time1=time*25000;
	for(i=time1;i>=0;i--)
	{
		
	}
}

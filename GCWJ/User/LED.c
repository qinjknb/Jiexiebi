#include "stm32f10x.h"                  // Device header

int LED_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitTypeDef GPIO_Initstructrue;
	GPIO_Initstructrue.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Initstructrue.GPIO_Pin = GPIO_Pin_10;
	GPIO_Initstructrue.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_Initstructrue);
}	
	
void LED_ON (void)
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_10);
}

void LED_OFF (void)
{
	GPIO_SetBits(GPIOB,GPIO_Pin_10);
}

void LED_Toggle (void)
{
	if (GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_10) == Bit_RESET)
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_10);
	}
	else
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_10);
	}
}

#include "wdg.h"
#include "stm32f10x_gpio.h"
#include "wdginn.h"

void WATCHDog_Init(void)
{
		GPIO_InitTypeDef  GPIO_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOC, &GPIO_InitStructure);	
		GPIO_ResetBits(GPIOC,GPIO_Pin_0);
}

void WatchDog_Oper(void)
{
	if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_0)==TRUE){
		GPIO_ResetBits(GPIOC,GPIO_Pin_0);
//		GPIO_ResetBits(GPIOB,GPIO_Pin_11);
	}
	
	else{
		GPIO_SetBits(GPIOC,GPIO_Pin_0);
//		GPIO_SetBits(GPIOB,GPIO_Pin_11);
		
	}
}








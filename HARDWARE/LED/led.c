#include "header.h"
#include "sys.h"	

void LEDTimerConfig(void);



//LED IO初始化
void LED_Init(void)
{
 
	GPIO_InitTypeDef  GPIO_InitStructure;
 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);	 //使能PB,PE端口时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;				 //LED0-->PB.5 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(GPIOC, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.5
	LEDTimerConfig();
	
//	TIM_SetCompare1(TIM8, 0);
//	TIM_SetCompare2(TIM8, 0);
//	TIM_SetCompare3(TIM8, 0);
}

/** 灯带控制输出 **/


void LEDTimerConfig(void)
{
    
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;    
    TIM_OCInitTypeDef  TIM_OCInitStructure;  
	    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);    
	
    TIM_TimeBaseStructure.TIM_Period = 1000;	//1M=1000000/1000=1000(Hz) 2ms
    TIM_TimeBaseStructure.TIM_Prescaler = 71; //72000000/72=1M 
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;                               
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;      
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);    
      
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;   
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                             
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;  
		TIM_OCInitStructure.TIM_Pulse = 100;//周期为2000,占空比25% 1/4高电平 3/4低电平
		TIM_OC1Init(TIM8, &TIM_OCInitStructure);
    TIM_OC2Init(TIM8, &TIM_OCInitStructure); // OC2 is for BLUE LED                        
    TIM_OC3Init(TIM8, &TIM_OCInitStructure); //                      

  	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);	
		TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);	
		TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);	
          
    TIM_ARRPreloadConfig(TIM8, ENABLE);   
		TIM_CtrlPWMOutputs(TIM8,ENABLE);
    TIM_Cmd(TIM8, ENABLE);      
}




#include "stm32f10x.h"
#include "key.h"
#include "sys.h" 
#include "delay.h"
#include "timerapp.h"
/*Initialize ports for Power, Motor Output, 24V output, Laser Power, PAD Power, and PC Power*/
void ENSWITCH_Init(void)
{
	u16 cnt;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC,GPIO_Pin_7);  // enable 12V1
	GPIO_ResetBits(GPIOC,GPIO_Pin_8);	 // enable 12V2
	GPIO_ResetBits(GPIOC,GPIO_Pin_9);		// enable 12V3
	GPIO_ResetBits(GPIOC,GPIO_Pin_2);		// enable 3.3v output
	GPIO_SetBits(GPIOC,GPIO_Pin_0);		// enable 5V output
	RESET_Motor();
	for (cnt=0; cnt<0xFFF; cnt++)
		;
	RESET_24VOUT();
	for (cnt=0; cnt<0xFFF; cnt++)
		;
	RESET_Power();
}

void RESET_Motor(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/*根据开关的步骤来设置初始化的状态*/
	GPIO_ResetBits(GPIOB,GPIO_Pin_10);
	gGlobal.m_PowerCoreStatus.m_EN_Motor=0;
}

void RESET_24VOUT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/*根据开关的步骤来设置初始化的状态*/
	GPIO_ResetBits(GPIOB,GPIO_Pin_11);
	gGlobal.m_PowerCoreStatus.m_EN_24V=0;
}

void RESET_Power(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/*根据开关的步骤来设置初始化的状态*/
	GPIO_ResetBits(GPIOA,GPIO_Pin_6);
	gGlobal.m_PowerCoreStatus.m_EN_Power=0;

}

void Detection_Motor(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;	
		EXTI_InitTypeDef EXTI_InitStructure;
//		NVIC_InitTypeDef NVIC_InitStructure;
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
		GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); 
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource10);
		
		EXTI_InitStructure.EXTI_Line=EXTI_Line10;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
	 gGlobal.m_PowerCoreStatus.m_EN_Motor=1;
//		EXTI_ClearITPendingBit(EXTI_Line10);
//	  dELAY_ms(1000);
//	  EXTI->IMR|=1<<10;
		


}

void Detection_24VOUT(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;	
		EXTI_InitTypeDef EXTI_InitStructure;

	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
		GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); 
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource11);
		
		EXTI_InitStructure.EXTI_Line=EXTI_Line11;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
		EXTI_ClearITPendingBit(EXTI_Line11);
	  gGlobal.m_PowerCoreStatus.m_EN_24V=1;
		

}

void Detection_Power(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;	
		EXTI_InitTypeDef EXTI_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
		GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); 
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource6);
		
		EXTI_InitStructure.EXTI_Line=EXTI_Line6;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 

		NVIC_Init(&NVIC_InitStructure); 
		EXTI_ClearITPendingBit(EXTI_Line6);
		gGlobal.m_PowerCoreStatus.m_EN_Power=1;
	
}



void KEY_Init(void) //IO初始化
{ 
 	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//使能PORTA时钟
	//初始化 WK_UP-->GPIOA.0	  下拉输入
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA0设置成输入，默认下拉	  
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA15
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); 
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource15);
	
	EXTI_InitStructure.EXTI_Line=EXTI_Line15;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 

  NVIC_Init(&NVIC_InitStructure); 
	EXTI_ClearITPendingBit(EXTI_Line15);


}
//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
//0，没有任何按键按下

u8 KEY_Scan(u8 mode)
{	 
	static u8 key_up=1;//按键按松开标志
  u32 cnt_delay;
	if(KEY0 != gGlobal.m_KEYstatus)
	{
		for (cnt_delay = 0 ;cnt_delay<=0x1FFFF; )
			cnt_delay++;
		key_up=KEY0;
		if(key_up!=gGlobal.m_KEYstatus && gGlobal.m_KEYstatus==1)  // 1----> 0 shutdown
			return KEY0_ShutDown;
		else if (key_up!=gGlobal.m_KEYstatus && gGlobal.m_KEYstatus==0)
			return KEY0_PowerUp;
		else
			return 0;// 无按键按下
	}  
	return 0;
 	
}


 void EXTI15_10_IRQHandler(void)
  {	
		/* Power Switch Detection*/
		if(EXTI_GetITStatus(EXTI_Line15)!=RESET)
			{
				gGlobal.m_KEYLock = KEY_Scan(1);
				EXTI_ClearITPendingBit(EXTI_Line15);
				EXTI_ClearFlag(EXTI_Line15);
				if (gGlobal.m_KEYLock != 0){
					EXTI->IMR &= ~(1<<15); //屏蔽line15上的中断
				}
		}
		 
		/* FLT detection*/
		if(EXTI_GetITStatus(EXTI_Line13)!=RESET)
		{
			EXTI_ClearITPendingBit(EXTI_Line13);
			EXTI_ClearFlag(EXTI_Line13);
			gGlobal.m_PTKFLAG.wFLAG_CHARGE =SET;
//			EXTI->IMR &= ~(1<<13);
			gGlobal.m_PTKNUM.err_CHARGE ++;
			if (gGlobal.m_PTKNUM.err_CHARGE>65535)
				gGlobal.m_PTKNUM.err_CHARGE=0;
		}
		/* Motor Dectection*/
		if(EXTI_GetITStatus(EXTI_Line10)!=RESET)
		{
			EXTI_ClearITPendingBit(EXTI_Line10);
			EXTI_ClearFlag(EXTI_Line10);
			EXTI->IMR &= ~(1<<10);
			RESET_Motor();
			gGlobal.m_PTKFLAG.wFLAG_MOTOR = 1;

		}		
		
		/* 24VOUT detection*/
		if(EXTI_GetITStatus(EXTI_Line11)!=RESET)
		{
			EXTI_ClearITPendingBit(EXTI_Line11);
			EXTI_ClearFlag(EXTI_Line11);
			EXTI->IMR &= ~(1<<11);
			RESET_24VOUT();
			gGlobal.m_PTKFLAG.wFLAG_24VOUT=SET;
		}		
  }

	
	/*Battery detection*/
void EXTI9_5_IRQHandler(void){
	if(EXTI_GetITStatus(EXTI_Line6)!=RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line6);
		EXTI_ClearFlag(EXTI_Line6);
		EXTI->IMR &= ~(1<<6); //disable EXTI_6
		RESET_Power();
		gGlobal.m_PTKFLAG.wFLAG_BATTERY=SET;

	}
}


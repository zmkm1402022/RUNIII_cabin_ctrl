#ifndef  __TIMERAPP_H
#define  __TIMERAPP_H

#include "stm32f10x.h"
void Timer1Init(void);
void TIM_DELAY_INIT(void);
void TasksectionInit(void);
void dELAY_us(uint16_t nus);
void dELAY_ms(uint16_t ums);
u32 dTime_Counting(u32 startingtime, u32 endingtime);
#endif


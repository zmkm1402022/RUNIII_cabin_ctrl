#include "exDCMotor.h"
#include "header.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "event_groups.h"
void DoorStateCheck(uint8_t ID, uint8_t m_dir, uint8_t mode);
void Locker_Running(u8 dir, u16 dutycycle);
int LockerChannelSelect(MODE lk_mode);
extern TimerHandle_t FaultProcTimer_Handle;
extern EventGroupHandle_t EventGroupHandle; 
u32 speed;
void Locker_Switching_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;			
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
	LOWERLOCKER = 0;
	UPPERLOCKER = 0;

}

/*
 *monitor the operating status of doors and lifter 
1. in single door mode, doorEndingSuccess = 1 is to identify a successful operation of a door.
2. in dual door mode, doorEndingSuccess = 2 is to identify a successful operation
*/
void MotorProcessMonitoring(void)
{
	static uint8_t locp_num =0, uocp_num=0;
	if (gGlobal.m_CAN.door_ready ==1 && gGlobal.m_CAN.door_msg_result == 0)
	{
		if(gGlobal.m_Status.Upperdoor_OCPFLAG == 0)
		{
			if(gGlobal.m_OCPData.currentCH1 > gGlobal.m_MOTORUpper.threshold)
			{
				uocp_num ++;
				if(uocp_num >=2)
				{
					DoorSingleMode_Running(BRAKE, 0,1);
					gGlobal.m_Status.Upperdoor_OCPFLAG =1;
					gGlobal.m_stack.doorEndingSuccess = 3;
					gGlobal.m_stack.door_interval = 10;
					uocp_num =0;
					gGlobal.m_Status.Upperdoor_OCPErr_CNT =0;
					gGlobal.m_stack.door_timetostart = gGlobal.m_LocalTime;
					xEventGroupSetBits( (EventGroupHandle_t) EventGroupHandle, (EventBits_t) FAULTTRIG_EVENTBIT);
					if(DEBUG == 2)
						printf("Warning: the OCP process of upper motor is triggered");				
				}
			}
		}
		
		if(gGlobal.m_Status.Lowerdoor_OCPFLAG == 0)
		{
			if(gGlobal.m_OCPData.currentCH3 > gGlobal.m_MOTORLower.threshold)
			{
				locp_num ++;
				if(locp_num >=2)
				{
					DoorSingleMode_Running(BRAKE, 0,2);
					gGlobal.m_Status.Lowerdoor_OCPFLAG =1;
					gGlobal.m_stack.doorEndingSuccess = 3;
					locp_num =0;
					gGlobal.m_Status.Lowerdoor_OCPErr_CNT =0;
					gGlobal.m_stack.door_timetostart = gGlobal.m_LocalTime;
					gGlobal.m_stack.door_interval = 1;
					xEventGroupSetBits( (EventGroupHandle_t) EventGroupHandle, (EventBits_t) FAULTTRIG_EVENTBIT);	
					if(DEBUG == 2)
						printf("Warning: the OCP process of lowermotor is triggered");					
				}
			}
		}

		if(!(Check1MSTick(gGlobal.m_LocalTime,gGlobal.m_stack.door_timetostart, gGlobal.m_stack.door_interval)))
			{
				if ( (!gGlobal.m_stack.doorEndingSuccess)||gGlobal.m_Status.Lowerdoor_OCPFLAG == 1||gGlobal.m_Status.Upperdoor_OCPFLAG == 1)
				{
					/* single mode */
					if (gGlobal.m_stack.operationID != 3)
					{
						/* code */
						DoorSingleMode_Running(BRAKE, 0,gGlobal.m_stack.operationID);
						if (gGlobal.m_stack.operationID == 1)
						{
							/* upperdoor timeout error */
							switch(gGlobal.m_stack.operationDIR)
							{
								case CLOCKWISE:
									if (UPPERDOOR_OPEN_IN)
									{
										gGlobal.m_MOTORUpper.err_flag = 0x41;
									}
									else
										gGlobal.m_MOTORUpper.err_flag = 3;  //timeout
								break;

								case COUNTERCLOCKWISE:
									if (UPPERDOOR_CLOSE_IN)
									{
										gGlobal.m_MOTORUpper.err_flag = 0x41;
									}
									else
										gGlobal.m_MOTORUpper.err_flag = 3; //timeout
								break;
							}
							gGlobal.m_Status.UPPERDOOR_ErrCNT ++;
						}
						else if (gGlobal.m_stack.operationID == 2)
						{
							/* lowerdoor timeout error */
							switch(gGlobal.m_stack.operationDIR)
							{
								case CLOCKWISE:
									if (LOWERDOOR_OPEN_IN)
									{
										/* code */
										gGlobal.m_MOTORLower.err_flag = 0x41;
									}
									else
										gGlobal.m_MOTORLower.err_flag = 3;
								break;

								case COUNTERCLOCKWISE:
									if (LOWERDOOR_CLOSE_IN)
									{
										/* code */
										gGlobal.m_MOTORLower.err_flag = 0x41;
									}
									else
										gGlobal.m_MOTORLower.err_flag = 3;
								break;

							}
							gGlobal.m_Status.LOWERDOOR_ErrCNT ++;								
						}
					}
					else
					{
						DoorDualMode_Running(BRAKE,0);
						switch(gGlobal.m_stack.operationDIR)
						{
							case CLOCKWISE:
								if (!UPPERDOOR_OPEN_IN)
								{
									gGlobal.m_MOTORUpper.err_flag = 3;
									gGlobal.m_Status.UPPERDOOR_ErrCNT ++;
								}
								else
									gGlobal.m_MOTORUpper.err_flag = 0;

								if (!LOWERDOOR_OPEN_IN)
								{
									/* code */
									gGlobal.m_MOTORLower.err_flag = 3;
									gGlobal.m_Status.LOWERDOOR_ErrCNT ++;
								}
								else
									gGlobal.m_MOTORLower.err_flag = 0;
							break;

							case COUNTERCLOCKWISE:
								if (!UPPERDOOR_CLOSE_IN)
								{
									gGlobal.m_MOTORUpper.err_flag = 3;
									gGlobal.m_Status.UPPERDOOR_ErrCNT ++;
								}
								else
									gGlobal.m_MOTORUpper.err_flag = 0;

								if (!LOWERDOOR_CLOSE_IN)
								{
									/* code */
									gGlobal.m_MOTORLower.err_flag = 3;
									gGlobal.m_Status.LOWERDOOR_ErrCNT ++;
								}
								else
									gGlobal.m_MOTORLower.err_flag = 0;
							break;
						}
					}
					if(DEBUG == 1)
						printf("Error: timeout during door operation, ID=%d\r\n",gGlobal.m_stack.operationID);
					gGlobal.m_CAN.door_msg_result = 2;
					uocp_num=0;
					locp_num=0;

				 }
			}

		else if (gGlobal.m_stack.doorEndingSuccess==1 && gGlobal.m_stack.operationID !=3)
		{
			/*  */
			gGlobal.m_CAN.door_msg_result = 1;
			if (gGlobal.m_stack.operationID == 1)
			{
				gGlobal.m_MOTORUpper.err_flag =0;
			}
			else if (gGlobal.m_stack.operationID == 2)
			{
				/* code */
				gGlobal.m_MOTORLower.err_flag =0;
			}
			if (gGlobal.m_stack.operationDIR == COUNTERCLOCKWISE)
			{
				if((gGlobal.m_stack.operationID ==1 && UPPERDOOR_CLOSE_IN == 1)\
						|| (gGlobal.m_stack.operationID ==2 && LOWERDOOR_CLOSE_IN == 1))
						LockerLatch_Config(gGlobal.m_stack.operationID );
			
				else if(DEBUG == 1)
					printf("error: the door is closed, but the sensor is not correctly touched\r\n");
			}
		}
		/* in dual mode operation, gGlobal.m_stack.doorEndingSuccess == 2 is the sign to identify a successful operation*/
		else if (gGlobal.m_stack.operationID ==3)
		{
			if ( gGlobal.m_stack.doorEndingSuccess == 2)
			{
				gGlobal.m_MOTORUpper.err_flag =0;
				gGlobal.m_MOTORLower.err_flag =0;
				gGlobal.m_CAN.door_msg_result = 1;
				if (UPPERDOOR_CLOSE_IN)
				{
					/* code */
					LockerLatch_Config(1);
				}
				if (LOWERDOOR_CLOSE_IN)
				{
					/* code */
					LockerLatch_Config(2);
				}
			}
		  else if(DEBUG == 1)
				printf(".");
		}
	}

	if ( gGlobal.m_CAN.lifter_ready == 1 && gGlobal.m_MOTORLifter.status == RUNNING )
	{
		/* TIMEOUT */
		if(!(Check1MSTick(gGlobal.m_LocalTime,gGlobal.m_MOTORLifter.timetostart, 25000)))
		{
			LIFTERTIMERDISABLE;
			gGlobal.m_MOTORLifter.err_flag =2;
			gGlobal.m_CAN.lifter_msg_result = 2;
			gGlobal.m_Status.LIFTER_ErrCNT++;
			if(DEBUG == 1)
				printf("Error: timeout to operate the lifter\r\n");
		}
	
		else if (gGlobal.m_stack.lifterEndingSuccess == SET)
		{
			/* code */
			LIFTERTIMERDISABLE;
			gGlobal.m_MOTORLifter.interval = gGlobal.m_LocalTime - gGlobal.m_MOTORLifter.timetostart;
			gGlobal.m_CAN.lifter_msg_result = 1;
			gGlobal.m_MOTORLifter.err_flag =0;
			if(DEBUG == 1)
			{
				printf("Note:time duration of the lifter operation is %d seconds\r\n",gGlobal.m_MOTORLifter.interval);
				printf("Note:rotating step is %d \r\n",gGlobal.m_stack.operationDoorAcc.stepper_pulse_cnt);
			}
		}
	}
	
	if(gGlobal.m_Status.Upperdoor_OCPFLAG == 1 || gGlobal.m_Status.Lowerdoor_OCPFLAG == 1)
	{
		if(gGlobal.m_OCPData.currentCH1 > gGlobal.m_MOTORUpper.threshold)
			gGlobal.m_MOTORUpper.err_OCPcnt++;
		if(gGlobal.m_OCPData.currentCH3 > gGlobal.m_MOTORLower.threshold)
			gGlobal.m_MOTORLower.err_OCPcnt++;
		if(DEBUG == 2)
			printf("Warning: the uocp_num= %d\r\n",gGlobal.m_Status.Lowerdoor_OCPErr_CNT);
		if(gGlobal.m_MOTORUpper.err_OCPcnt >= 2)
		{
			DoorSingleMode_Running(BRAKE, 0,1);
			gGlobal.m_MOTORUpper.err_OCPcnt =0;
			gGlobal.m_MOTORUpper.err_flag = 6 ;
			
			if(gGlobal.m_Status.Upperdoor_OCPErr_CNT <3)
				xEventGroupSetBits( (EventGroupHandle_t) EventGroupHandle, (EventBits_t) FAULTTRIG_EVENTBIT);
		}
		else if(gGlobal.m_MOTORLower.err_OCPcnt >=2)
		{
			DoorSingleMode_Running(BRAKE, 0,2);
			gGlobal.m_MOTORLower.err_OCPcnt =0;
			gGlobal.m_MOTORLower.err_flag = 6;
			if(gGlobal.m_Status.Lowerdoor_OCPErr_CNT <3)
				xEventGroupSetBits( (EventGroupHandle_t) EventGroupHandle, (EventBits_t) FAULTTRIG_EVENTBIT);		
		}
		
		if(!(Check1MSTick(gGlobal.m_LocalTime,gGlobal.m_stack.door_timetostart, 40000)))
		{
			if(gGlobal.m_Status.Upperdoor_OCPFLAG == 1)
			{
				DoorSingleMode_Running(BRAKE, 0,1) ;
				gGlobal.m_MOTORUpper.err_flag = 5 ;   // timeout during fault processing, the fault is not correctly solved within 40s
				if(gGlobal.m_Status.Upperdoor_OCPErr_CNT <3)
					xEventGroupSetBits( (EventGroupHandle_t) EventGroupHandle, (EventBits_t) FAULTTRIG_EVENTBIT);
				else
					gGlobal.m_Status.Upperdoor_OCPFLAG = 0xEE;
			}
			if(gGlobal.m_Status.Lowerdoor_OCPFLAG == 1)
			{
				DoorSingleMode_Running(BRAKE, 0,2) ;
				gGlobal.m_MOTORLower.err_flag = 5;
				if(gGlobal.m_Status.Lowerdoor_OCPErr_CNT <3)
					xEventGroupSetBits( (EventGroupHandle_t) EventGroupHandle, (EventBits_t) FAULTTRIG_EVENTBIT);
				else
					gGlobal.m_Status.Upperdoor_OCPFLAG = 0xEE;
			}			
		
		}
		else if(gGlobal.m_stack.doorEndingSuccess > 0)
		{
			if(gGlobal.m_Status.Upperdoor_OCPFLAG == 1)
			{
				if(gGlobal.m_stack.operationDIR == CLOCKWISE && UPPERDOOR_OPEN_IN == 1)
				{
					if(DEBUG == 2)
					printf("UPPERDOOR_OPEN_IN ending success number= %d\r\n",gGlobal.m_stack.doorEndingSuccess);
					gGlobal.m_Status.Upperdoor_OCPFLAG = 0xFF;
					gGlobal.m_stack.doorEndingSuccess --;		
					
				}
				else if(gGlobal.m_stack.operationDIR == COUNTERCLOCKWISE && UPPERDOOR_CLOSE_IN == 1)
				{
					gGlobal.m_Status.Upperdoor_OCPFLAG = 0xFF;
					if(DEBUG == 2)
						printf("UPPERDOOR_CLOSE_IN = %d\r\n",gGlobal.m_stack.doorEndingSuccess);		
					gGlobal.m_stack.doorEndingSuccess --;		
								
				}
			}
			if(gGlobal.m_Status.Lowerdoor_OCPFLAG == 1)
			{
				if(gGlobal.m_stack.operationDIR == CLOCKWISE && LOWERDOOR_OPEN_IN == 1)
				{
					gGlobal.m_Status.Lowerdoor_OCPFLAG = 0xFF;
					gGlobal.m_stack.doorEndingSuccess --;				
				}	
				else if(gGlobal.m_stack.operationDIR == COUNTERCLOCKWISE && LOWERDOOR_CLOSE_IN == 1)
				{
					gGlobal.m_Status.Lowerdoor_OCPFLAG = 0xFF;
					gGlobal.m_stack.doorEndingSuccess --;						
				}
			}			
		}
	}

}


/*
1. deal with the faulty conditions while motor is operating.
 */

void MotorFaultMonitoring(void)
{
	uint8_t fDIR;
	uint32_t fPWM, dtime;
	BaseType_t errcode;
	if (gGlobal.m_Status.Upperdoor_OCPFLAG == 1)
	{
		gGlobal.m_Status.Upperdoor_OCPErr_CNT++;
		
		if(gGlobal.m_stack.operationDIR == CLOCKWISE)
				fDIR = COUNTERCLOCKWISE;
		else if (gGlobal.m_stack.operationDIR == COUNTERCLOCKWISE)
				fDIR = CLOCKWISE;
		fPWM = 2*PWM_DUTYFACTOR_1;
		TIM_ITConfig(TIM1,TIM_IT_CC1,DISABLE ); 
		gGlobal.m_MOTORUpper.status = RUNNING;
		DoorSingleMode_Running(fDIR,fPWM ,1);
		if(gGlobal.m_Status.Upperdoor_OCPErr_CNT <=2)
			delay_xms (1500);
		else
		{
			if (fDIR == CLOCKWISE)
			{
				while (!UPPERDOOR_OPEN_IN) ;
			}
			else
				while (!UPPERDOOR_CLOSE_IN) ;
		}
				
		DoorSingleMode_Running(BRAKE, 0,1) ;
		if(gGlobal.m_Status.Upperdoor_OCPErr_CNT <3)
		{

			if(gGlobal.m_Status.Upperdoor_OCPErr_CNT >=2)
				dtime = 10000;
			else
				dtime = 5000;
			errcode = xTimerChangePeriod( (TimerHandle_t) FaultProcTimer_Handle, dtime, 10);

			errcode = xTimerStart(FaultProcTimer_Handle,10) ;
			if(errcode == pdFAIL)
			{
				if(DEBUG == 2)
					printf("Warning: fail to start the timer\r\n");
			}	
			else if(DEBUG == 2)
				printf("Warning: succeed to start the timer\r\n");
		}


	}
	else if (gGlobal.m_Status.Lowerdoor_OCPFLAG == 1)
	{
		gGlobal.m_Status.Lowerdoor_OCPErr_CNT++;
		if(gGlobal.m_stack.operationDIR == CLOCKWISE)
				fDIR = COUNTERCLOCKWISE;
		else if (gGlobal.m_stack.operationDIR == COUNTERCLOCKWISE)
				fDIR = CLOCKWISE;
		fPWM = 2*PWM_DUTYFACTOR_1;
		TIM_ITConfig(TIM1,TIM_IT_CC1,DISABLE ); 
		gGlobal.m_MOTORLower.status = RUNNING;
		DoorSingleMode_Running(fDIR,fPWM ,2);
		if(gGlobal.m_Status.Lowerdoor_OCPErr_CNT <=2)
			delay_xms (1500);
		else
		{
			if (fDIR == CLOCKWISE)
			{
				while (!LOWERDOOR_OPEN_IN) ;
			}
			else
				while (!LOWERDOOR_CLOSE_IN) ;
		}
				
		DoorSingleMode_Running(BRAKE, 0,2) ;
		
		if(gGlobal.m_Status.Lowerdoor_OCPErr_CNT <3)
		{

			if(gGlobal.m_Status.Lowerdoor_OCPErr_CNT >=2)
				dtime = 10000;
			else
				dtime = 5000;
			errcode = xTimerChangePeriod( (TimerHandle_t) FaultProcTimer_Handle, dtime, 10);

			errcode = xTimerStart(FaultProcTimer_Handle,10) ;
			if(errcode == pdFAIL)
			{
				if(DEBUG == 2)
					printf("Warning: fail to start the timer\r\n");
			}	
			else if(DEBUG == 2)
				printf("Warning: succeed to start the timer\r\n");
		}
	}
}
/* 
void Motor_CTRL_Init(void) is to initialize motors, which include two door motors, two locker motors and one stepper motor
1. TIM3 is for two locker motor
2. TIM5 is for two door motors
3. TIM2 is for the lifter motor
*/



void Motor_CTRL_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	/*config the Lifter PWM output ports*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO , ENABLE);
	GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;			
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	
    /* configure the LOCKER PWM output ports */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO , ENABLE);
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 | GPIO_Pin_5;			
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/*Motor Enable output ports*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;			//PC3 Upper/Lower motor enable pin
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_15;  //PB15 Lifter enable pin
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/*Upper Motor and Lower Motor output ports*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO , ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;			
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	
    /* Configure TIM3 for Locker motor */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);
  TIM_TimeBaseStructure.TIM_Period  = 400;//  output 50KHz = 160, 20KHz = 400, 
  TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock/8000000)-1;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode =  TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 100;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_CtrlPWMOutputs(TIM3,ENABLE);
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_Cmd(TIM3, DISABLE);

    /* Configure TIM2 for Lifter motor */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
  TIM_TimeBaseStructure.TIM_Period  = 1000 * PWM_FREQ;//
  TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock/1000000)-1;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode =  TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = PWM_DUTYFACTOR_20;
//	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
//	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_CtrlPWMOutputs(TIM2,ENABLE);
	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_ITConfig(TIM2,TIM_IT_CC4,ENABLE ); 
	TIM_Cmd(TIM2, DISABLE);

  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

	/* Configure TIM5 for both Upper/Lower motors  */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5 , ENABLE);
  TIM_TimeBaseStructure.TIM_Period  = TIM3_PERIOD_CNT;//
  TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock/TIM3_TICK_HZ)-1;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode =  TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = DUTYCYCLE;
	
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);
	TIM_OC3Init(TIM5, &TIM_OCInitStructure);
	TIM_OC4Init(TIM5, &TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);
	
	TIM_CtrlPWMOutputs(TIM5,ENABLE);
	TIM_ARRPreloadConfig(TIM5, ENABLE);
	TIM_Cmd(TIM5, DISABLE);
}

/*
Parameters: 
1. dir refers to the operating direction
2. dutycycle refers to the PWM dutycycle
3. door refers to the door ID being targeted
 */
void DoorSingleMode_Running(u8 dir, int16_t dutycycle,u8 door)
{
	if (dutycycle > PWM_DUTYFACTOR_90)
		dutycycle = PWM_DUTYFACTOR_90;
	else if (dutycycle < PWM_DUTYFACTOR_1)
		dutycycle = PWM_DUTYFACTOR_1;
	switch (door)
	{
		case 1:   // controlling the operation of uppermotor
			
		if (gGlobal.m_MOTORUpper.status == RUNNING)
			{
				/* code */
				if (dir == CLOCKWISE)
				{
					TIM_SetCompare1(TIM5,dutycycle);
					TIM_SetCompare2(TIM5,0);
				}
				else if (dir == COUNTERCLOCKWISE)
				{
					TIM_SetCompare1(TIM5,0);
					TIM_SetCompare2(TIM5,dutycycle);	
				}
				else if (dir == BRAKE)
				{
					gGlobal.m_MOTORUpper.status = BRAKE;
					UPPERMOTORBRAKE;
				}
			}
		break;
		
		case 2:		// controlling the operation of lowermotor
			if (gGlobal.m_MOTORLower.status == RUNNING)
			{
				/* code */
				if (dir == CLOCKWISE)
				{
					TIM_SetCompare3(TIM5,dutycycle);
					TIM_SetCompare4(TIM5,0);
				}
				else if (dir == COUNTERCLOCKWISE)
				{
					TIM_SetCompare3(TIM5,0);
					TIM_SetCompare4(TIM5,dutycycle);	
				}
				else if (dir == BRAKE)
				{
					LOWERMOTORBRAKE;
					gGlobal.m_MOTORLower.status = BRAKE;
				}				
			}
		break;
	}
}

void DoorDualMode_Running(u8 dir, int16_t dutycycle)
{
	DoorSingleMode_Running(dir, dutycycle, 1);
	DoorSingleMode_Running(dir, dutycycle, 2);
}

void Lifter_Running(u8 dir, int16_t dutycycle)
{
	if(gGlobal.m_MOTORLifter.status == RUNNING){
		if (dir == CLOCKWISE )
		{
			TIM_SetCompare3(TIM2,dutycycle);
			TIM_SetCompare4(TIM2,0);
		}
		else if (dir == COUNTERCLOCKWISE )
		{
			TIM_SetCompare3(TIM2,0);
			TIM_SetCompare4(TIM2,dutycycle);
		}
		else if (dir == BRAKE)
		{
			LIFTERBRAKE;
			gGlobal.m_MOTORLifter.status = BRAKE;
		}
		else if ( dir == IDLE)
		{
			LIFTERIDLE;
			gGlobal.m_MOTORLifter.status = IDLE;
		}
	}
}

void Locker_Running(u8 dir, u16 dutycycle)
{
	if (dir == CLOCKWISE){
		TIM_SetCompare1(TIM3,dutycycle);
		TIM_SetCompare2(TIM3,0);
	}
	else if (dir == COUNTERCLOCKWISE)
	{
		TIM_SetCompare1(TIM3,0);
		TIM_SetCompare2(TIM3,dutycycle);
	}
	else if (dir == IDLE)
	{
		LOCKERSTANDBY;
	}
	else if (dir == BRAKE)
	{
		TIM_SetCompare1(TIM3,500);
		TIM_SetCompare2(TIM3,500);	
	}
}
/*
Initilize an action to door motors, which include a process of checking the initial position, going back
to the origin (if needed), and starting opening/closing process.

 */
void DoorEnable_Config(void)
{
	u8 errd[4] = {0xFF, 0xFF, 0xFF, 0xFF};
	switch(gGlobal.m_stack.operationID)  // ID=1--> upperdoor; ID =2 --> Lowerdoor; ID=3 --> dual door
	{
		/*single operation mode*/
		default:
			gGlobal.m_MOTORUpper.err_flag = 0;
			gGlobal.m_MOTORLower.err_flag = 0;
			gGlobal.m_CAN.operation_mode = 1;   //operation_mode
			if(DEBUG == 1)
				printf("Note: start to check the initial status of doors!\r\n");
			DoorStateCheck(gGlobal.m_stack.operationID, gGlobal.m_stack.operationDIR, 1);
			if(gGlobal.m_stack.operationID ==1)
			{
				if (gGlobal.m_MOTORUpper.err_flag !=0 \
					|| gGlobal.m_LOCKERUpper.err_flag !=0 || gGlobal.m_CAN.operation_mode == 0)  
				{
					/* fail to pass the state check for upper door */
					gGlobal.m_CAN.door_msg_result = 1;
					gGlobal.m_Status.UPPERDOOR_ErrCNT ++;
					if(DEBUG == 1)
						printf("Warning: Exiting Upperdoor operation, uppermotor_flag = %d, upperlocker_flag = %d\r\n",\
							gGlobal.m_MOTORUpper.err_flag, gGlobal.m_LOCKERUpper.err_flag );
					
					if(gGlobal.m_stack.operationDIR == COUNTERCLOCKWISE)
					{
						LockerLatch_Config(ID_01);
					}
					errd[0] = gGlobal.m_MOTORUpper.err_flag;
					errd[2] = gGlobal.m_LOCKERUpper.err_flag;
					AppResultRes(CMD_LOCKER_DOOR,gGlobal.m_CAN.door_msg_result , errd, 4);
					gGlobal.m_CAN.door_msg_result = 0;
//					printf("a\r\n");
					break;
				}		
			}
			else if(gGlobal.m_stack.operationID ==2 )
			{
				if (gGlobal.m_MOTORLower.err_flag !=0 \

						|| gGlobal.m_LOCKERLower.err_flag !=0 || gGlobal.m_CAN.operation_mode == 0 )
				{
					/* fail to pass the state check for lower door*/
					gGlobal.m_CAN.door_msg_result = 1;
					gGlobal.m_Status.LOWERDOOR_ErrCNT ++;
					if(DEBUG == 1)
						printf("Warning: Exiting Lowerdoor operation, uppermotor_flag = %d, upperlocker_flag = %d\r\n",\
							gGlobal.m_MOTORLower.err_flag, gGlobal.m_LOCKERLower.err_flag );
					if(gGlobal.m_stack.operationDIR == COUNTERCLOCKWISE)
					{
						LockerLatch_Config(ID_02);
					}
					errd[1] = gGlobal.m_MOTORLower.err_flag;
					errd[3] = gGlobal.m_LOCKERLower.err_flag;
					AppResultRes(CMD_LOCKER_DOOR,gGlobal.m_CAN.door_msg_result , errd, 4);
					gGlobal.m_CAN.door_msg_result = 0;
					break;
				}			
			}

			if (gGlobal.m_stack.operationID ==1 )
				gGlobal.m_MOTORUpper.status = RUNNING;
			else if(gGlobal.m_stack.operationID == 2)
				gGlobal.m_MOTORLower.status = RUNNING;
			gGlobal.m_stack.doorEndingSuccess = 0;
			gGlobal.m_stack.operationDoorAcc.realtimeIntCnt =0;
			gGlobal.m_stack.operationDoorAcc.accTime = 80;
			gGlobal.m_stack.operationDoorAcc.deaccTime = 200 ;
			gGlobal.m_stack.operationDoorAcc.accStep = 20 ;
			gGlobal.m_stack.operationDoorAcc.deaccStep =10 ;
			gGlobal.m_CAN.door_ready =1;
			gGlobal.m_stack.operationDUTY = PWM_DUTYFACTOR_0;
			if(DEBUG ==1)
				printf("Warning: Singlemode the Lowerdoor starts to open or close!\r\n");
			gGlobal.m_stack.door_timetostart = gGlobal.m_LocalTime;
			gGlobal.m_stack.door_interval = 12000;
			DoorSingleMode_Running(gGlobal.m_stack.operationDIR,gGlobal.m_stack.operationDUTY ,gGlobal.m_stack.operationID);
			TIM_ITConfig(TIM1,TIM_IT_CC1,ENABLE ); 
			TIM_Cmd(TIM1, ENABLE);

		break;

		/* dual operation mode*/
		case 3:
			gGlobal.m_stack.doorEndingSuccess = 0;  // a sign to identify the operation. 2 = dual mode, 1 = single mode.
			gGlobal.m_MOTORUpper.err_flag = 0;
			gGlobal.m_MOTORLower.err_flag = 0;
			/*check the upperdoor state. */
			DoorStateCheck(1, gGlobal.m_stack.operationDIR,2);
			/*check the lowerdoor state*/
			DoorStateCheck(2, gGlobal.m_stack.operationDIR,2);
//			/*check the lifter state*/
//			DoorStateCheck(3, 0, 2);
			
			if (gGlobal.m_LOCKERLower.err_flag !=0 || gGlobal.m_MOTORLower.err_flag !=0 || gGlobal.m_MOTORUpper.err_flag !=0 || gGlobal.m_MOTORLower.err_flag !=0)
			{
				/* code */
				gGlobal.m_CAN.door_msg_result = 2;
				gGlobal.m_Status.LOWERDOOR_ErrCNT ++;
				gGlobal.m_Status.UPPERDOOR_ErrCNT ++;
				if(DEBUG ==1)
					printf("Error: Exiting dual mode operation during the initializing stage\r\n");
				errd[0] = gGlobal.m_MOTORUpper.err_flag;
				errd[2] = gGlobal.m_LOCKERUpper.err_flag;
				errd[1] = gGlobal.m_MOTORLower.err_flag;
				errd[3] = gGlobal.m_LOCKERLower.err_flag;
				AppResultRes(CMD_LOCKER_DOOR,gGlobal.m_CAN.door_msg_result , errd, 4);
				gGlobal.m_CAN.door_msg_result = 0;
				break;
			}
			else if(gGlobal.m_stack.doorEndingSuccess == 2)
			{
				gGlobal.m_CAN.door_msg_result = 1;
				errd[0] = gGlobal.m_MOTORUpper.err_flag;
				errd[2] = gGlobal.m_LOCKERUpper.err_flag;
				errd[1] = gGlobal.m_MOTORLower.err_flag;
				errd[3] = gGlobal.m_LOCKERLower.err_flag;
				AppResultRes(CMD_LOCKER_DOOR,gGlobal.m_CAN.door_msg_result , errd, 4);
				gGlobal.m_CAN.door_msg_result = 0;	
				gGlobal.m_CAN.door_ready =0;
				if(DEBUG ==1)
					printf("Success: both doors are at the targeted position\r\n");		
				break;				
			}
			
			gGlobal.m_CAN.operation_mode = 1;
			
			gGlobal.m_stack.operationDoorAcc.realtimeIntCnt =0;
			gGlobal.m_stack.operationDoorAcc.accTime = 300;
			gGlobal.m_stack.operationDoorAcc.deaccTime = 200 ;
			gGlobal.m_stack.operationDoorAcc.accStep = 20 ;
			gGlobal.m_stack.operationDoorAcc.deaccStep =10 ;
			gGlobal.m_stack.operationDUTY = PWM_DUTYFACTOR_0;
			if(DEBUG == 1)
				printf("Warning: Dualmode the door starts to open or close!\r\n");
			gGlobal.m_stack.door_timetostart = gGlobal.m_LocalTime;
			gGlobal.m_stack.door_interval = 12000;
			gGlobal.m_CAN.door_ready =1;
//			DoorSingleMode_Running(gGlobal.m_stack.operationDIR,gGlobal.m_stack.operationDUTY ,gGlobal.m_stack.operationID);
			DoorDualMode_Running(gGlobal.m_stack.operationDIR, gGlobal.m_stack.operationDUTY );
			TIM_ITConfig(TIM1,TIM_IT_CC1,ENABLE ); 
			TIM_Cmd(TIM1, ENABLE);

		break;
	
	}
}

/* Initilize an action to lifters*/
void LifterEnable_Config(void)
{
	uint8_t opsition =0;
	
	switch(gGlobal.m_stack.operationDoorAcc.stepper_destination)
	{
		case 1:  // top position
			if (LIFTER_TOP_IN == 1 )
				opsition =1;
			else if(LIFTER_BOTTOM_IN == 1)
			{
				gGlobal.m_MOTORLifter.dir = COUNTERCLOCKWISE; //moving upwards
				gGlobal.m_stack.operationDoorAcc.stepper_distance = DISTANCE_IN_STEPS;
				gGlobal.m_stack.operationAccTimeforLifter = ACC_TIME;
				
			}
			else
			{
				gGlobal.m_MOTORLifter.dir = COUNTERCLOCKWISE; //moving upwards
				gGlobal.m_stack.operationDoorAcc.stepper_distance = DISTANCE_IN_STEPS/2;
				gGlobal.m_stack.operationAccTimeforLifter = ACC_TIME;			
			}
		break;
		
		case 3:	// bottom position
			if (LIFTER_BOTTOM_IN == 1 )
				opsition =1;
			else if(LIFTER_TOP_IN == 1)
			{
				gGlobal.m_MOTORLifter.dir = CLOCKWISE; //moving downwards
				gGlobal.m_stack.operationDoorAcc.stepper_distance = DISTANCE_IN_STEPS;
				gGlobal.m_stack.operationAccTimeforLifter = ACC_TIME;
			}
			else
			{
				gGlobal.m_MOTORLifter.dir = CLOCKWISE; //moving downwards
				gGlobal.m_stack.operationDoorAcc.stepper_distance = DISTANCE_IN_STEPS/2;
				gGlobal.m_stack.operationAccTimeforLifter = ACC_TIME;			
			}			
			
		break;
		
		case 2:	// middle position
			if(LIFTER_TOP_IN == 1)
			{
				gGlobal.m_MOTORLifter.dir = CLOCKWISE; //moving downwards
				gGlobal.m_stack.operationDoorAcc.stepper_distance = DISTANCE_IN_STEPS/2;
				gGlobal.m_stack.operationAccTimeforLifter = ACC_TIME;				
			}
			else if(LIFTER_BOTTOM_IN == 1)
			{
				gGlobal.m_MOTORLifter.dir = COUNTERCLOCKWISE; //moving upwards
				gGlobal.m_stack.operationDoorAcc.stepper_distance = DISTANCE_IN_STEPS/2;
				gGlobal.m_stack.operationAccTimeforLifter = ACC_TIME;			
			}
			else if(gGlobal.m_MOTORLifter.err_flag == 0)
				opsition =1;
			else
			{
				switch(gGlobal.m_stack.operationDoorAcc.stepper_des_prev)
				{
					case 1:
						gGlobal.m_MOTORLifter.dir = CLOCKWISE; //moving downwards
						gGlobal.m_stack.operationDoorAcc.stepper_distance = DISTANCE_IN_STEPS/2;
						gGlobal.m_stack.operationAccTimeforLifter = ACC_TIME;						
					break;
					
					case 3:
						gGlobal.m_MOTORLifter.dir = COUNTERCLOCKWISE; //moving upwards
						gGlobal.m_stack.operationDoorAcc.stepper_distance = DISTANCE_IN_STEPS/2;
						gGlobal.m_stack.operationAccTimeforLifter = ACC_TIME;							
					break;
					
					case 2:
						opsition =1;
					break;
				}
			
			}
		break;
	}
	if (opsition ==1)
	{
		/* no more action to be done, as the lifter is already on positon */
		 
		gGlobal.m_MOTORLifter.status = BRAKE;
		gGlobal.m_MOTORLifter.err_flag = 0;
		gGlobal.m_CAN.lifter_msg_result = 1;
		printf("Warning: Lifter is on the targeted position\r\n");
		return;
	}
	if(gGlobal.m_MOTORLifter.dir == CLOCKWISE)
	{
		STEPPER_DIR = 0;  // moving downwards
	}
	else
		STEPPER_DIR = 1;	// moving upwards
	gGlobal.m_stack.operationSpeedforLifter = 200;
	speed = (gGlobal.m_stack.operationSpeedforLifter - 1)*100/gGlobal.m_stack.operationAccTimeforLifter;  //加速度
	speed = speed/100;  // at
	gGlobal.m_stack.StepperAccDeltaAT = (u16) speed/10;   // rpm * 10
	gGlobal.m_MOTORLifter.timetostart = gGlobal.m_LocalTime;
	gGlobal.m_stack.operationDoorAcc.stepper_pulse_cnt = 0;  //统计给出的脉冲个数
	gGlobal.m_stack.operationDoorAcc.stepper_time_cnt =0;
	gGlobal.m_stack.operationIntCNTforLifter =0;  // 统计加速区间的时间
	gGlobal.m_MOTORLifter.status = RUNNING;
	gGlobal.m_MOTORLifter.interval = 20000;
	gGlobal.m_stack.operationIntCNTforLifter = 0;
	gGlobal.m_stack.lifterEndingSuccess = 0;
	gGlobal.m_MOTORLifter.dutycycle = 10;
	gGlobal.m_MOTORLifter.timetostart = gGlobal.m_LocalTime;
	Lifter_Speed_Update(gGlobal.m_MOTORLifter.dutycycle);
	TIM_Cmd(TIM2, ENABLE);
	TIM_ITConfig(TIM1,TIM_IT_CC2,ENABLE ); 
//	TIM1->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
//	if ((TIM1->CR1 & TIM_CR1_CEN) == 0)
//	{
//		/* code */
//		TIM_Cmd(TIM1, ENABLE);
//	}
}

 /*
void LockerLatch_Config(uint8_t lockerID)
1) if the door needs to be closed while the upperlocked is in locked position, upperlocker should be released before operating the motor
2) same applied to the lowerlocker in similar situations
*/
void LockerLatch_Config(uint8_t lockerID)
{
	if (lockerID == 1 && UPPERLOCKER_LOCKED_IN != 1)
	{
		
		/* release the upperlocker */
		gGlobal.m_LOCKERUpper.err_flag = 0;
		UPPERLOCKER_ENABLE;
		LOWERLOCKER_DISABLE;
		gGlobal.m_LOCKERUpper.timetostart = gGlobal.m_LocalTime;
		Locker_Running(COUNTERCLOCKWISE, 250);
		LOCKERTIMERENABLE;
		while(UPPERLOCKER_LOCKED_IN != 1 )
		{
			if(DEBUG == 1 && gGlobal.m_CAN.door_ready == 0)
				printf("Current: locker channel current = %d mA\r\n", gGlobal.m_OCPData.currentCH4);
			if(gGlobal.m_LocalTime < gGlobal.m_LOCKERUpper.timetostart)
				gGlobal.m_LOCKERUpper.interval = 0xFFFFFFFF + gGlobal.m_LocalTime - gGlobal.m_LOCKERUpper.timetostart;
			else
				gGlobal.m_LOCKERUpper.interval = gGlobal.m_LocalTime - gGlobal.m_LOCKERUpper.timetostart;
			
			if (gGlobal.m_LOCKERUpper.interval >= 2000)  // 2s = 0x8954400
			{
				/* code */
				if (UPPERLOCKER_RELEASED_IN == 1)
				{
					/* if the PB13 is not released (and PB1 is not triggered )within 2s, this situation could be motor problem */
					gGlobal.m_LOCKERUpper.err_flag = 3;
				}
				/* if the PB13 is released but PB1 is not triggered within 2s*/
				else
					gGlobal.m_LOCKERUpper.err_flag = 2; //timeout
				if(DEBUG == 1)
					printf(" Error: one of the upperlocker switch has probelm. \r\n");
				break;
			}
			if (gGlobal.m_Status.LOCKER_OCPFLAG == 1)
			{
				gGlobal.m_LOCKERUpper.err_flag = 10;   // 10 = over current error flag
				LOCKERSTANDBY;
				if(DEBUG == 1 && gGlobal.m_CAN.door_ready == 0)
				{
					printf("Error: the locker channel is tripped off due to overcurrent issue!\r\n");
					printf("Error: corresponding error code is %d!\r\n",gGlobal.m_LOCKERUpper.err_flag);
				}
				break;
			}

		}
		delay_us(20000);
		Locker_Running(BRAKE, 500);
		if(DEBUG == 1 && gGlobal.m_CAN.door_ready == 0)
			printf("Time Interval: the duration of locker operation is %d ms\r\n", gGlobal.m_LOCKERUpper.interval);
		/* update the upperlocker status*/
		if (UPPERLOCKER_RELEASED_IN == 1)
		{
			/* code */
			gGlobal.m_Status.UPPERLOCKER_STATUS = 2;
		}
		else if (UPPERLOCKER_LOCKED_IN == 1)
		{
			/* code */
			gGlobal.m_Status.UPPERLOCKER_STATUS = 1;
		}
		else
			gGlobal.m_Status.UPPERLOCKER_STATUS = 3;
		
		if (gGlobal.m_LOCKERUpper.err_flag != 0)
		{
			/* code */
			gGlobal.m_Status.LOCKER_ErrCNT ++;
			if (DEBUG == 1 && gGlobal.m_CAN.door_ready == 0)
				printf("Error: The upperlocker fails to latch in, errorcode is %d\r\n",gGlobal.m_LOCKERUpper.err_flag);
		}
	}
	else if (lockerID == 2 && LOWERLOCKER_LOCKED_IN != 1)
	{
		/* code */
		/* release the upperlocker */
		gGlobal.m_LOCKERLower.err_flag = 0;
		LOWERLOCKER_ENABLE;
		UPPERLOCKER_DISABLE;
		Locker_Running(COUNTERCLOCKWISE, 250);
		LOCKERTIMERENABLE;
		gGlobal.m_LOCKERLower.timetostart = gGlobal.m_LocalTime;
		while(LOWERLOCKER_LOCKED_IN != 1 )
		{
			if(gGlobal.m_LocalTime < gGlobal.m_LOCKERLower.timetostart)
				gGlobal.m_LOCKERLower.interval = 0xFFFFFFFF + gGlobal.m_LocalTime -gGlobal.m_LOCKERLower.timetostart;
			else
				gGlobal.m_LOCKERLower.interval = gGlobal.m_LocalTime - gGlobal.m_LOCKERLower.timetostart;			
			if (gGlobal.m_LOCKERLower.interval >2000)  // 2s = 0x8954400
			{
				/* code */
				if (LOWERLOCKER_RELEASED_IN == 1)
				{
					/* if the PB13 is not released (and PB1 is not triggered )within 2s, this situation could be motor problem */
					gGlobal.m_LOCKERLower.err_flag = 3;
				}
				/* if the PB13 is released but PB1 is not triggered within 2s*/
				else 
					gGlobal.m_LOCKERLower.err_flag = 2;  //timeout
				break;
			}
			if (gGlobal.m_Status.LOCKER_OCPFLAG == 1)
			{
				gGlobal.m_LOCKERUpper.err_flag = 10;   // 10 = over current error flag
				LOCKERSTANDBY;
				if(DEBUG == 1 && gGlobal.m_CAN.door_ready == 0)
				{
					printf("Error: the locker channel is tripped off due to overcurrent issue!\r\n");
					printf("Error: corresponding error code is %d!\r\n",gGlobal.m_LOCKERUpper.err_flag);
				}
				break;
			}
		}
		Locker_Running(BRAKE, 80);
		/* update the lowerlocker status*/
		if (LOWERLOCKER_LOCKED_IN == 1 )
		{
			/* code */
			gGlobal.m_Status.LOWERLOCKER_STATUS = 1;
		}
		else if (LOWERLOCKER_RELEASED_IN == 1)
		{
			/* code */
			gGlobal.m_Status.LOWERLOCKER_STATUS = 2;
		}
		else
			gGlobal.m_Status.LOWERLOCKER_STATUS =3;
		
		if (gGlobal.m_LOCKERLower.err_flag != 0)
		{
			/* code */
			gGlobal.m_Status.LOCKER_ErrCNT ++;
			if(DEBUG == 1 && gGlobal.m_CAN.door_ready == 0)
			{
				if(gGlobal.m_LOCKERLower.err_flag == 3 )
					printf("Error: the lowerlocker fails to lock into the door\r\n");
				else
					printf("Error: the lowerlocker timeout\r\n");
			}
		}
	}
	else if (lockerID == 1 && UPPERLOCKER_LOCKED_IN == 1)
	{
		/* code */
		gGlobal.m_Status.UPPERLOCKER_STATUS = 1;
	}
	else if (lockerID == 2 && LOWERLOCKER_LOCKED_IN == 1)
	{
		/* code */
		gGlobal.m_Status.LOWERLOCKER_STATUS =1;
	}
}


 /*
void LockerRelease_Config(uint8_t lockerID) 渚涘?ㄨ??ㄤ娇??
1) if the door needs to be closed while the upperlocked is in locked position, upperlocker should be released before operating the motor
2) same applied to the lowerlocker in similar situations
*/
void LockerRelease_Config(uint8_t lockerID)
{
	uint32_t count = 0;

	if (lockerID == 1)
	{
		
		/* release the upperlocker */
		gGlobal.m_LOCKERUpper.err_flag = 0;
		UPPERLOCKER_ENABLE;
		LOWERLOCKER_DISABLE;
		Locker_Running(CLOCKWISE, 150);
		gGlobal.m_LOCKERUpper.interval =0;
		gGlobal.m_LOCKERUpper.timetostart = gGlobal.m_LocalTime;
		while(UPPERLOCKER_RELEASED_IN != 1 )
		{

			if(gGlobal.m_LocalTime < gGlobal.m_LOCKERUpper.timetostart)
				gGlobal.m_LOCKERUpper.interval = 0xFFFFFFFF + gGlobal.m_LocalTime - gGlobal.m_LOCKERUpper.timetostart;
			else
				gGlobal.m_LOCKERUpper.interval = gGlobal.m_LocalTime - gGlobal.m_LOCKERUpper.timetostart;
				
			if (gGlobal.m_LOCKERUpper.interval >= 2000)  // 2s = 0x8954400
			{
				/* timeout code */
					gGlobal.m_LOCKERUpper.err_flag = 2;
				break;
			}
			if (gGlobal.m_Status.LOCKER_OCPFLAG == 1)
			{
				gGlobal.m_LOCKERUpper.err_flag = 10;   // 10 = over current error flag
				gGlobal.m_Status.LOCKER_OCPFLAG = 0 ;
				LOCKERSTANDBY;
				if(DEBUG == 1 && gGlobal.m_CAN.door_ready == 0)
				{
					printf("Error: the locker channel is tripped off due to overcurrent issue!\r\n");
					printf("Error: corresponding error code is %d!\r\n",gGlobal.m_LOCKERUpper.err_flag);
				}
				break;
			}
		}
		Locker_Running(BRAKE, 80);
		if(DEBUG ==1 && gGlobal.m_CAN.door_ready == 0)
			printf("Time Interval: duration of locker operation is %d ms\r\n", gGlobal.m_LOCKERUpper.interval);
		if (UPPERLOCKER_RELEASED_IN == 1)
		{
			/* upperlocker status is updated as 2 (released position)*/
			gGlobal.m_Status.UPPERLOCKER_STATUS = 2;
		}
		else if (UPPERLOCKER_LOCKED_IN == 1)
		{
			/* upperlocker status is updated as 1 (locked position) */
			gGlobal.m_Status.UPPERLOCKER_STATUS = 1; 
			}
		else
			gGlobal.m_Status.UPPERLOCKER_STATUS = 3; 
		if (gGlobal.m_LOCKERUpper.err_flag != 0)
		{
			/* code */
			gGlobal.m_Status.LOCKER_ErrCNT ++;
			if(DEBUG ==1 && gGlobal.m_CAN.door_ready == 0)
				printf("Error: The upperlocker fails to release, errorcode is %d\r\n",gGlobal.m_LOCKERUpper.err_flag);
		}
	}
	else if (lockerID == 2)
	{
		/* code */
		/* release the lowerlocker */
		gGlobal.m_LOCKERLower.err_flag = 0;
		LOWERLOCKER_ENABLE;
		UPPERLOCKER_DISABLE;
		Locker_Running(CLOCKWISE, 150);
		LOCKERTIMERENABLE;
		while(LOWERLOCKER_RELEASED_IN != 1 )
		{
			count ++;
//			printf(".");
			if (count >= 0x154400)  // 2s = 0x8954400
			{
				/* code */
				if (LOWERLOCKER_LOCKED_IN == 1)
				{
					/* if the PB13 is not released (and PB1 is not triggered )within 2s, this situation could be motor problem */
					gGlobal.m_LOCKERLower.err_flag = 1;
				}
				/* if the PB13 is released but PB1 is not triggered within 2s*/
				else 
					gGlobal.m_LOCKERLower.err_flag = 2;  //timeout
				break;
			}
			if (gGlobal.m_Status.LOCKER_OCPFLAG == 1)
			{
				gGlobal.m_LOCKERLower.err_flag = 10;
				gGlobal.m_Status.LOCKER_OCPFLAG = 0 ;

				LOCKERSTANDBY;
				if (DEBUG == 1 && gGlobal.m_CAN.door_ready == 0)
				{
					printf("Error: the locker channel is tripped off due to overcurrent issue!\r\n");
					printf("Error: corresponding error code is %d!\r\n",gGlobal.m_LOCKERUpper.err_flag);				
				}
				break;
			}
		}
		Locker_Running(BRAKE, 80);
		if (LOWERLOCKER_LOCKED_IN == 1 )
		{
			gGlobal.m_Status.LOWERLOCKER_STATUS = 1;
		}
		else if (LOWERLOCKER_RELEASED_IN == 1)
		{
			gGlobal.m_Status.LOWERLOCKER_STATUS = 2;
		}
		else
			gGlobal.m_Status.LOWERLOCKER_STATUS =3;
		if (gGlobal.m_LOCKERLower.err_flag != 0)
		{
			/* code */
			gGlobal.m_Status.LOCKER_ErrCNT ++;
			if(DEBUG ==1 && gGlobal.m_CAN.door_ready == 0)
				printf("Error: The lowerlocker fails to release, errorcode is %d\r\n",gGlobal.m_LOCKERLower.err_flag );
		}
	}

}

/*
Parameters:
ID refers to the door to be operated
m_dir refers to CW/CCW
mode refers to the single mode (=1) or dual mode (=2)
*/
void DoorStateCheck(uint8_t ID, uint8_t m_dir, uint8_t mode)
{
	uint8_t sensorfbk1 = 0xFF, sensorfbk2 = 0xFF,onpositionflag;
	onpositionflag =0;
	LockerRelease_Config(ID);
	if (ID ==1 )
	{
		sensorfbk1 = UPPERDOOR_OPEN_IN;
		sensorfbk2 =  UPPERDOOR_CLOSE_IN; 
		gGlobal.m_MOTORUpper.status = RUNNING;
	}
	else if (ID ==2 )
	{
		sensorfbk1 = LOWERDOOR_OPEN_IN;
		sensorfbk2 = LOWERDOOR_CLOSE_IN;
		gGlobal.m_MOTORLower.status = RUNNING;
	}
	/* */
	if (m_dir ==0x11 && sensorfbk1 == 1)
	{
		/* code */
		onpositionflag =1;  
		if ( ID ==1 )
		{
			/* code */
			gGlobal.m_Status.UPPERDOOR_STATUS = 1;
		}
		else if (ID ==2 )
		{
			/* code */
			gGlobal.m_Status.LOWERDOOR_STATUS = 1;
		}
	}
	else if (m_dir ==0x12 && sensorfbk2 == 1)
	{
		/* code */
		onpositionflag =1;
		if ( ID ==1 )
		{
			/* code */
			gGlobal.m_Status.UPPERDOOR_STATUS = 2;
		}
		else if (ID ==2 )
		{
			/* code */
			gGlobal.m_Status.LOWERDOOR_STATUS = 2;
		}		
	}
	if (onpositionflag ==1 )
	{
		if (mode ==1)
		{
			/* code */
			gGlobal.m_CAN.operation_mode = 0;  // clear the operation mode (running/idle)
			gGlobal.m_CAN.door_ready = 0;  //ready for receiving next command
		}
		if (ID == 1)
		{
			/* code */
			gGlobal.m_MOTORUpper.status = BRAKE;
			if(mode == 2)
				gGlobal.m_stack.doorEndingSuccess ++;
			if(DEBUG == 1)
				printf("NOTE:the upperdoor is already on position\r\n");
		}
		else if ( ID == 2)
		{
			/* code */
			gGlobal.m_MOTORLower.status = BRAKE;
			if(mode == 2)
				gGlobal.m_stack.doorEndingSuccess ++;
			if(DEBUG ==1)
				printf("NOTE:the lowerdoor is already on position\r\n");
		}
		
		return;
	}
/* when the door is in-between, neither at the closed/opened position, it is going to go through the back to origin. Otherwise, no such process*/
	if (sensorfbk2 == 0 && sensorfbk1 == 0)
	{

		if ( gGlobal.m_LOCKERUpper.err_flag != 0 && ID == 1)
		{
			/* code */
			DoorbacktoOrigin(ID, m_dir);
		}
		else if (gGlobal.m_LOCKERLower.err_flag != 0 && ID == 2)
		{
			/* code */
			DoorbacktoOrigin(ID, m_dir);
		}
		
	}
	/* 
	Check the lifter postion
	1) in single mode, the lifter should be in the middle.
	2) in dual mode, the lifter should be in the bottom.
	*/
//	if (ID == 1 || ID == 2)
//	{
//		/* single mode  */
//		if (LIFTER_TOP_IN != 1 && mode == 1)
//		{
//			/* code */
//			gGlobal.m_CAN.lifter_ready = 1;
//			gGlobal.m_CAN.lifter_msg_result = 0;
//			gGlobal.m_stack.lifterEndingSuccess=0;
//			gGlobal.m_MOTORLifter.status = RUNNING;
//			gGlobal.m_MOTORLifter.dir = CLOCKWISE;   // CLOCKWISE
//			LifterEnable_Config();
//		}
//	}
//	else if (ID == 3)
//	{
//		/* dual mode */
//		if (LIFTER_BOTTOM_IN != 1 && mode == 2)
//		{
//			/* code */
//			gGlobal.m_CAN.lifter_ready = 1;
//			gGlobal.m_CAN.lifter_msg_result = 0;
//			gGlobal.m_stack.lifterEndingSuccess=0;
//			gGlobal.m_MOTORLifter.status = RUNNING;
//			gGlobal.m_MOTORLifter.dir = COUNTERCLOCKWISE;  // COUNTERCLOCKWISE
//			LifterEnable_Config();
//		}
//	}
}
/**
This function is used to control the doors which return to its initial position, it can only control the upper or lower motor in single mode (not suitable for dual mode),
this function does not include the controlling of lockers, meanwhile, the output voltage for motors is fixed as well (dutycyle = 20%)

The initial position is defined under two conditions:
(1) the firmware startup condition: the upperdoor/lowerdoor should go back to the closed position and be locked well.
(2) after the startup while controlling the motor to open/close the door, it should be at the opposite original position, this is considered for sync the position and PWM controlling.
In other cases, if the door is not at the targeted position with  the corresponding switch be touched, this is a problem of control and/or hardware.

[param 1] o_ID specifies the targeted door to be operated; 1 = upperdoor, 2 = lowerdoor
[param 2] o_dir specifies the cycling direction; attention should be paid for choosing the right direction. 
								CLOCKWISE = open door; COUNTERCLOCKWISE = close door;
[return]	0 is pass; -1 means failure.
*/
int DoorbacktoOrigin(uint8_t o_ID, uint8_t o_dir)
{
		int32_t time;
		uint8_t  o_sensor,vdir = 0;
	  int err;
		if(o_dir == CLOCKWISE)
			vdir = COUNTERCLOCKWISE;
		else
			vdir = CLOCKWISE;
		switch(o_ID)
		{
			case 1: // doorID =1
				gGlobal.m_MOTORUpper.timetostart = gGlobal.m_LocalTime;
				gGlobal.m_MOTORUpper.interval = 5000;
				gGlobal.m_MOTORUpper.dutycycle = PWM_DUTYFACTOR_20;
				gGlobal.m_MOTORUpper.status = RUNNING;
				DoorSingleMode_Running(vdir, gGlobal.m_MOTORUpper.dutycycle,o_ID);
				o_sensor = 0;
				while( !o_sensor )
				{
					if (vdir == COUNTERCLOCKWISE)
					{
						o_sensor =UPPERDOOR_CLOSE_IN;
					}
					else
						o_sensor = UPPERDOOR_OPEN_IN;
					if(gGlobal.m_ADCstatus == 1)
					{
						gGlobal.m_ADCstatus = 0;
						FetchingCurrentValue();
						DMA_Cmd(DMA1_Channel1, ENABLE);
						ADC_Cmd(ADC1, ENABLE);		
						if(DEBUG == 1)
							printf("CH1: %d\t\t\tCH2: %d\t\t\tCH3: %d\t\t\tCH4: %d\r\n",gGlobal.m_OCPData.currentCH1,\
																			gGlobal.m_OCPData.currentCH2,\
																			gGlobal.m_OCPData.currentCH3,\
																			gGlobal.m_OCPData.currentCH4);				

					}
					time = gGlobal.m_LocalTime - gGlobal.m_MOTORUpper.timetostart;
					if (time < 0)
					{
						/* code */
						time = 0xFFFFFFFF - time;
					}
					if( time >= gGlobal.m_MOTORUpper.interval && o_sensor == 0)  //timeout occurs here
					{
						gGlobal.m_MOTORUpper.err_flag = 0x03;
						UPPERMOTORBRAKE;
						gGlobal.m_MOTORUpper.status = BRAKE;
						err = -1;
						break;
					}
					else if ( o_sensor == 1)
					{
						/* code */
						gGlobal.m_MOTORUpper.err_flag = 0;
						UPPERMOTORBRAKE;
						gGlobal.m_MOTORUpper.status = BRAKE;
						err = 0;
						break;
					}

				}
			break;
			
			case 2: // doorID =2
				gGlobal.m_MOTORLower.timetostart = gGlobal.m_LocalTime;
				gGlobal.m_MOTORLower.interval = 5000;
				gGlobal.m_MOTORLower.dutycycle = PWM_DUTYFACTOR_20;
				if(o_dir == CLOCKWISE)
					o_dir = COUNTERCLOCKWISE;
				gGlobal.m_MOTORLower.status = RUNNING;
				DoorSingleMode_Running(o_dir, gGlobal.m_MOTORLower.dutycycle,o_ID);
				o_sensor = 0;
				while( !o_sensor )
				{
					if (vdir == COUNTERCLOCKWISE)
					{
						o_sensor =LOWERDOOR_CLOSE_IN;
					}
					else
						o_sensor = LOWERDOOR_OPEN_IN;
					if(gGlobal.m_ADCstatus == 1)
					{
						gGlobal.m_ADCstatus = 0;
						FetchingCurrentValue();
						DMA_Cmd(DMA1_Channel1, ENABLE);
						ADC_Cmd(ADC1, ENABLE);		
						if(DEBUG == 1)
							printf("CH1: %d\t\t\tCH2: %d\t\t\tCH3: %d\t\t\tCH4: %d\r\n",gGlobal.m_OCPData.currentCH1,\
																			gGlobal.m_OCPData.currentCH2,\
																			gGlobal.m_OCPData.currentCH3,\
																			gGlobal.m_OCPData.currentCH4);				

					}
					time = gGlobal.m_LocalTime - gGlobal.m_MOTORLower.timetostart;
					if (time < 0)
					{
						/* code */
						time = 0xFFFFFFFF - time;
					}
					if( time >= gGlobal.m_MOTORLower.interval && o_sensor == 0) //timeout appears
					{
						gGlobal.m_MOTORLower.err_flag = 0x03;
						gGlobal.m_MOTORLower.status = BRAKE;
						LOWERMOTORBRAKE;
						err = -1;
						break;
					}
					else if ( o_sensor == 1)
					{
						/* code */
						gGlobal.m_MOTORLower.err_flag = 0;
						gGlobal.m_MOTORLower.status = BRAKE;
						LOWERMOTORBRAKE;
						err =0;
						break;
					}

				}
			break;		
		}

		return err;
}


/*
This function is only used for control lockers.
[param 1] l_ID specifies the target
[param 2] l_dir specifies the direction. CW = unlock, CCW = lock
*/
int LockerbacktoOrigin(uint8_t l_ID, uint8_t l_dir)
{
	u32 timecnt = 0;
	int err;
	if ( l_ID == 1) /* upperlocker*/
	{
		/* code */
		err = LockerChannelSelect(LOCKER_UP);
		
		if(err != RESET)
			return -1;
		Locker_Running(l_dir, 80);
		if ( l_dir == CLOCKWISE)
		{
			/* if it's clockwise operation, wait until the  UPPERLOCKER_RELEASED_IN being triggered*/
			while ( UPPERLOCKER_RELEASED_IN !=1)   
			{
				timecnt++;
//				if(timecnt > 0xFFFFF)
//					return -1;
			}
		}
		else if (l_dir == COUNTERCLOCKWISE)
		{
			/* code */
			while ( UPPERLOCKER_LOCKED_IN !=1)
			{
				timecnt++;
//				if(timecnt > 0xFFFFF)
//					return -1;
			}
		}
		Locker_Running(BRAKE, 20);
		
	}
	else if (l_ID == 2)  // lowerlocker
	{
		/* code */
		LockerChannelSelect(LOCKER_DOWN);
		Locker_Running(l_dir, 80);
		if ( l_dir == CLOCKWISE)
		{
			/* code */
			while ( LOWERLOCKER_RELEASED_IN !=1)
			{
				timecnt++;
				if(timecnt > 0xFFFFF)
					return -1;
			}
		}
		else if (l_dir == COUNTERCLOCKWISE)
		{
			/* code */
			while ( LOWERLOCKER_LOCKED_IN !=1)
			{
				timecnt++;
				if(timecnt > 0xFFFFF)
					return -1;
			}
		}
		Locker_Running(BRAKE, 20);
	}
	else if (DEBUG == 1)
		printf("Error: The input parameters are not correct!\n");
	return 0;
}

/*
This function is to select the driving output for the targeted lockers
[param] lk_mode: LOCKER_UP is to drive the upperlocker only
								 LOCKER_DOWN is to drive the lowerlocker only
								 LOCKER_DUAL is to drive dual lockers simultanously
[output] 0 stands for OK; -1 stands for fail
*/

int LockerChannelSelect(MODE lk_mode)
{
	if (lk_mode == LOCKER_UP)
	{
		UPPERLOCKER = 0;
		LOWERLOCKER = 1;
	}
	else if(lk_mode == LOCKER_DOWN)
	{
		UPPERLOCKER = 1;
		LOWERLOCKER = 0;
	}
	else if(lk_mode == LOCKER_DUAL)
	{
		UPPERLOCKER = 0;
		LOWERLOCKER = 0;	
	}
	else
		return -1;
	return 0;
}


void Lifter_Speed_Update(u16 rpm)
{
	u32 pps;
	pps = rpm*STEPPER_XIFENG/6; // pulses/s * 10
	pps = STEPPER_TIM_CLK*1000000/pps;
	pps *= 100;
	TIM_SetCompare4(TIM2, (uint16_t) pps/2);
	TIM_SetAutoreload(TIM2, (uint16_t) pps);
}

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_CC4))
	{
			/* code */
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
		TIM_ClearFlag(TIM2, TIM_IT_CC4);
		gGlobal.m_stack.operationDoorAcc.stepper_pulse_cnt++;
		if(gGlobal.m_stack.operationDoorAcc.stepper_destination == 2)
		{
			if(gGlobal.m_stack.operationDoorAcc.stepper_pulse_cnt >= 27600)
				gGlobal.m_stack.lifterEndingSuccess = 1;
		}
	}
}

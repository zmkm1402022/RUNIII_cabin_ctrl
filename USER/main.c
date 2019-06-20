#include "sys.h"
#include <string.h>
#include <stdlib.h>
#include "header.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "event_groups.h"
#include "semphr.h"
#include "SEGGER_SYSVIEW.h"

void PWRInit(void);
void InitGlobalSet(void);
#define PWRSwitch 			PBout(2)

GlobalSet gGlobal;

BaseType_t errcode;

EventGroupHandle_t EventGroupHandle; 
SemaphoreHandle_t BinarySemaphore_USART;
SemaphoreHandle_t BinarySemaphore_ADCDATA;
EventGroupHandle_t EventGroupHandle;

#define FaultProcTimer_ID 1
TimerHandle_t FaultProcTimer_Handle;
void FaultProcTimerCallBack(TimerHandle_t xTimer);

#define START_TASK_PRIO 1
#define START_STK_SIZE	256
TaskHandle_t StartTask_Handler;
void start_task(void * pvParameters);

/* Falut Processing task*/
#define FAULTPROC_TASK_PRIO 10
#define FAULTPROC_STK_SIZE 128
TaskHandle_t FaultProcTask_Handler;
void faultproc_task(void * pvParameters);

/* Monitor task*/
#define MONITOR_TASK_PRIO 2
#define MONITOR_TASK_SIZE 128
TaskHandle_t MonitorTask_Handler;
void monitor_task(void * pvParameters);

/* Message task*/
#define MSG_TASK_PRIO 2
#define MSG_TASK_SIZE 128
TaskHandle_t MsgTask_Handler;
void message_task(void * pvParameters);

/* Door task */
#define DOOR_TASK_PRIO 5
#define DOOR_TASK_SIZE 128
TaskHandle_t DoorTask_Handler;
void door_task(void * pvParameters);

/* Lifter task*/
#define LIFTER_TASK_PRIO 5
#define LIFTER_TASK_SIZE 128
TaskHandle_t LifterTask_Handler;
void lifter_task(void * pvParameters);


/* Locker task*/
#define LOCKER_TASK_PRIO 5
#define LOCKER_TASK_SIZE 128
TaskHandle_t LockerTask_Handler;
void locker_task(void * pvParameters);


/* USART task*/
#define USART_TASK_PRIO 8
#define USART_STK_SIZE 128
TaskHandle_t UsartTask_Handler;
void usart_decode_task(void * pvParameters);

/* broadcast task*/
#define BRDCAST_TASK_PRIO 1
#define BRDCAST_STK_SIZE 128
TaskHandle_t BrdcastTask_Handler;
void broadcast_task(void * pvParameters);

/* touching sensor task*/
#define TOUCHING_TASK_PRIO 1
#define TOUCHING_STK_SIZE 128
TaskHandle_t TouchingTask_Handler;
void touching_task(void * pvParameters);


/* ADC processing task*/
#define ADC_TASK_PRIO 6
#define ADC_STK_SIZE  256
TaskHandle_t ADCTask_Handler;
void adc_processing_task(void * pvParameters);

u32 num = 0;
float *VelocityTab = NULL;
u32 AccelStep;
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4	 
	SEGGER_SYSVIEW_Conf();
	delay_init();
	uart_init(115200);
	VoltSampling_Init();
	Timer1Init();  // initialize TIM1 which is used to accelerate motors
	OCPChannel_Init();  // define the motor OCP ports
	Locker_Switching_Init();   // define the relay ports
	ONPOSITIONChannel_Init();
	if(!DS18B20_Init())
		printf ("Warning: a DS1820 has been detected!\r\n");
	else
		printf ("Warning: failing to detect the DS1820!\r\n");
	Motor_CTRL_Init();
	ULMOTORTIMERENABLE;
	LIFTERTIMERDISABLE;
	LOCKERSTANDBY;
	LOCKERTIMERENABLE;
	CAN_Configuration();
	PWRInit();
	PWRSwitch = 1;
	STEPPER_DIR = 0;
	ULMOTORENABLE =1; //enable the door motor channels
	InitGlobalSet();
	gGlobal.m_MOTORUpper.threshold = 500;
	gGlobal.m_MOTORLower.threshold = 500;
	xTaskCreate((TaskFunction_t )start_task,            
							(const char*    )"start_task",         
							(uint16_t       )START_STK_SIZE,        
							(void*          )NULL,                  
							(UBaseType_t    )START_TASK_PRIO,       
							(TaskHandle_t*  )&StartTask_Handler);                
	vTaskStartScheduler();          
}


BOOL Check1MSTick(DWORD dwCurTickCount, DWORD dwStart, DWORD DelayTime)
{
    if (dwCurTickCount >= dwStart)
        return ((dwCurTickCount - dwStart) < DelayTime);
    else
        return ((0xFFFFFFFF - dwStart + dwCurTickCount) < DelayTime);
}


void start_task(void * pvParameters)
{
	taskENTER_CRITICAL();
   EventGroupHandle = xEventGroupCreate();
	if(DEBUG == 1)
		{
			if( EventGroupHandle == NULL )
			{
				printf("error: fail to create the event group\r\n");
			}
			else
			{
				printf("note:  create the event group successfully\r\n");
			}
		}
	BinarySemaphore_USART = xSemaphoreCreateBinary();
	BinarySemaphore_ADCDATA = xSemaphoreCreateCounting(20,0);
	EventGroupHandle = xEventGroupCreate();
		
	errcode = 	xTaskCreate(	(TaskFunction_t	) usart_decode_task,
														(const char *		) "usart_task",
														(uint16_t				)	USART_STK_SIZE,
														(void *					) NULL,
														(UBaseType_t		) USART_TASK_PRIO,
														(TaskHandle_t *	) &UsartTask_Handler ) ;
														
														
							xTaskCreate(	(TaskFunction_t	) adc_processing_task,
														(const char *		) "adc_task",
														(uint16_t				)	USART_STK_SIZE,
														(void *					) NULL,
														(UBaseType_t		) ADC_TASK_PRIO,
														(TaskHandle_t *	) &ADCTask_Handler ) ;		

							xTaskCreate(	(TaskFunction_t	) locker_task,
														(const char *		) "locker_task",
														(uint16_t				)	LOCKER_TASK_SIZE,
														(void *					) NULL,
														(UBaseType_t		) LOCKER_TASK_PRIO,
														(TaskHandle_t *	) &LockerTask_Handler ) ;	
														
							xTaskCreate(	(TaskFunction_t	) message_task,
														(const char *		) "msg_task",
														(uint16_t				)	MSG_TASK_SIZE,
														(void *					) NULL,
														(UBaseType_t		) MSG_TASK_PRIO,
														(TaskHandle_t *	) &MsgTask_Handler ) ;		
														
							xTaskCreate(	(TaskFunction_t	) door_task,
														(const char *		) "door_task",
														(uint16_t				)	DOOR_TASK_SIZE,
														(void *					) NULL,
														(UBaseType_t		) DOOR_TASK_PRIO,
														(TaskHandle_t *	) &DoorTask_Handler ) ;	
														
							xTaskCreate(	(TaskFunction_t	) lifter_task,
														(const char *		) "lifter_task",
														(uint16_t				)	LIFTER_TASK_SIZE,
														(void *					) NULL,
														(UBaseType_t		) LIFTER_TASK_PRIO,
														(TaskHandle_t *	) &LifterTask_Handler ) ;	
														
							xTaskCreate(	(TaskFunction_t	) monitor_task,
														(const char *		) "monitor_task",
														(uint16_t				)	MONITOR_TASK_SIZE,
														(void *					) NULL,
														(UBaseType_t		) MONITOR_TASK_PRIO,
														(TaskHandle_t *	) &MonitorTask_Handler ) ;

							xTaskCreate(	(TaskFunction_t	) faultproc_task,
														(const char *		) "fault_task",
														(uint16_t				)	FAULTPROC_STK_SIZE,
														(void *					) NULL,
														(UBaseType_t		) FAULTPROC_TASK_PRIO,
														(TaskHandle_t *	) &MonitorTask_Handler ) ;
														
							FaultProcTimer_Handle = xTimerCreate( (const char *) "FaultProcTimer",
																										(TickType_t) 100,
																										(UBaseType_t) pdFALSE,
																										(void *) FaultProcTimer_ID,
																										(TimerCallbackFunction_t) FaultProcTimerCallBack );
	if(FaultProcTimer_Handle==NULL)
		printf("Warning: fail to create the one-shot timer\r\n");
	else
	{
		printf("Warning: succeed to create the one-shot timer\r\n");
	}
														
	vTaskDelete(StartTask_Handler);
	TIM_Cmd(TIM4, ENABLE);  // start the ADC;
	taskEXIT_CRITICAL();
}

void FaultProcTimerCallBack(TimerHandle_t xTimer)
{
		if(DEBUG == 2)
			printf("Warning: the timer is triggerred dir = %d\r\n", gGlobal.m_stack.operationDIR);
		gGlobal.m_stack.doorEndingSuccess = 0;
		
		
		if(gGlobal.m_Status.Upperdoor_OCPFLAG ==1)
		{
			gGlobal.m_MOTORUpper.status = RUNNING;
			DoorSingleMode_Running(gGlobal.m_stack.operationDIR,PWM_DUTYFACTOR_10,1);
			
		}
		else if (gGlobal.m_Status.Lowerdoor_OCPFLAG == 1)
		{
			gGlobal.m_MOTORLower.status = RUNNING;
			DoorSingleMode_Running(gGlobal.m_stack.operationDIR,PWM_DUTYFACTOR_10,2);
			
		}
	
}


void faultproc_task(void * pvParameters)
{
	while(1)
	{
		if(EventGroupHandle != NULL)
		{
			xEventGroupWaitBits( 	(EventGroupHandle_t) 	EventGroupHandle,
														(EventBits_t			 ) 	FAULTTRIG_EVENTBIT,
														(BaseType_t				 ) 	pdTRUE,
														(BaseType_t				 ) 	pdFALSE,
														(TickType_t				 ) 	portMAX_DELAY );	
			if(DEBUG == 2)
				printf("Warning: into the fault process task\r\n");
			MotorFaultMonitoring();
			
		}
		else
			vTaskDelay(20);
	}

}

void monitor_task(void * pvParameters)
{
	while(1)
	{
		MotorProcessMonitoring();
		vTaskDelay(200);	
	}
}

void lifter_task(void * pvParameters)
{
	while(1)
	{
		if(EventGroupHandle != NULL)
		{
			xEventGroupWaitBits( 	(EventGroupHandle_t) 	EventGroupHandle,
														(EventBits_t			 ) 	LIFTER_EVENTBIT,
														(BaseType_t				 ) 	pdTRUE,
														(BaseType_t				 ) 	pdFALSE,
														(TickType_t				 ) 	portMAX_DELAY );		
			LifterEnable_Config();
		}
		else
		{
			vTaskDelay(10);
		}
	}
}

void door_task(void * pvParameters)
{
	while(1)
	{
		if(EventGroupHandle != NULL)
		{
			xEventGroupWaitBits( 	(EventGroupHandle_t) 	EventGroupHandle,
														(EventBits_t			 ) 	DOOR_EVENTBIT,
														(BaseType_t				 ) 	pdTRUE,
														(BaseType_t				 ) 	pdFALSE,
														(TickType_t				 ) 	portMAX_DELAY );		
			DoorEnable_Config();
		}
		else
		{
			vTaskDelay(10);
		}
	}
}


void message_task(void * pvParameters)
{
	u8 err[4] ;
	while(1)
	{
		if (gGlobal.m_CAN.locker_ready == 1 && gGlobal.m_CAN.locker_msg_result != 0)
		{

			
			if(gGlobal.m_LOCKERUpper.status != 0){
//				dir = gGlobal.m_LOCKERUpper.status;
				err[0] = gGlobal.m_LOCKERUpper.err_flag;
				gGlobal.m_LOCKERUpper.err_flag =0;
				gGlobal.m_LOCKERUpper.status = 0;
			}
			else if (gGlobal.m_LOCKERLower.status !=0){
//				dir = gGlobal.m_LOCKERLower.status;
				err[0] = gGlobal.m_LOCKERLower.err_flag;
				gGlobal.m_LOCKERLower.err_flag =0;
				gGlobal.m_LOCKERLower.status =0;
			}
			AppResultRes(CMD_LOCKER_UPPERLOWER,gGlobal.m_CAN.locker_msg_result , err, 1);
			gGlobal.m_CAN.locker_msg_result =0;
			gGlobal.m_CAN.locker_ready =0;
		}
	if(gGlobal.m_CAN.door_ready == 1 && gGlobal.m_CAN.door_msg_result==3 &&\
		(gGlobal.m_Status.Upperdoor_OCPFLAG == 0xFF || gGlobal.m_Status.Upperdoor_OCPFLAG == 0xEE ||\
			gGlobal.m_Status.Lowerdoor_OCPFLAG == 0xFF || gGlobal.m_Status.Lowerdoor_OCPFLAG == 0xEE) )
	{
		err[0] = gGlobal.m_MOTORUpper.err_flag;
		err[2] = gGlobal.m_LOCKERUpper.err_flag;
		err[1] = gGlobal.m_MOTORLower.err_flag;
		err[3] = gGlobal.m_LOCKERLower.err_flag;			
		if(gGlobal.m_Status.Upperdoor_OCPFLAG == 0xFF || gGlobal.m_Status.Lowerdoor_OCPFLAG == 0xFF)
		{
				gGlobal.m_CAN.door_msg_result= 5;
		}
		else if (gGlobal.m_Status.Upperdoor_OCPFLAG == 0xEE || gGlobal.m_Status.Lowerdoor_OCPFLAG == 0xEE)
			gGlobal.m_CAN.door_msg_result= 6;
		AppResultRes(CMD_LOCKER_DOOR,gGlobal.m_CAN.door_msg_result , err, 4);
		gGlobal.m_CAN.door_msg_result = 0;
		gGlobal.m_CAN.door_ready = 0;
		gGlobal.m_stack.operationID =0 ;
		gGlobal.m_stack.operationDIR =0 ;
		gGlobal.m_CAN.operation_mode = 0;	
		if (gGlobal.m_Status.Upperdoor_OCPFLAG != 0)
			gGlobal.m_Status.Upperdoor_OCPFLAG = 0;
		if(gGlobal.m_Status.Lowerdoor_OCPFLAG != 0)
			gGlobal.m_Status.Lowerdoor_OCPFLAG = 0;
	}
	if (gGlobal.m_CAN.door_ready == 1 && (gGlobal.m_CAN.door_msg_result == 2 || gGlobal.m_CAN.door_msg_result == 1))
	{
		/* code */
		TIM_ITConfig(TIM1,TIM_IT_CC1,DISABLE ); 
		
		switch (gGlobal.m_stack.operationID)
		{
			case 1:
				err[0] = gGlobal.m_MOTORUpper.err_flag;
				err[2] = gGlobal.m_LOCKERUpper.err_flag;
				err[1] = 0xFF;
				err[3] = 0xFF;
			break;
				
			case 2:
				err[1] = gGlobal.m_MOTORLower.err_flag;
				err[3] = gGlobal.m_LOCKERLower.err_flag;
				err[2] = 0xFF;
				err[0] = 0xFF;
			break;
			
			case 3:
				err[0] = gGlobal.m_MOTORUpper.err_flag;
				err[2] = gGlobal.m_LOCKERUpper.err_flag;
				err[1] = gGlobal.m_MOTORLower.err_flag;
				err[3] = gGlobal.m_LOCKERLower.err_flag;
			break;
		
		}
		AppResultRes(CMD_LOCKER_DOOR,gGlobal.m_CAN.door_msg_result , err, 4);
		gGlobal.m_CAN.door_msg_result = 3;
		if( gGlobal.m_Status.Upperdoor_OCPFLAG == 0 && gGlobal.m_Status.Lowerdoor_OCPFLAG == 0)
		{
			gGlobal.m_CAN.door_ready = 0;
			gGlobal.m_stack.operationID =0 ;
			gGlobal.m_stack.operationDIR =0 ;
			gGlobal.m_CAN.operation_mode = 0;		
			gGlobal.m_CAN.door_msg_result = 0;
		}

	}
	if(gGlobal.m_CAN.lifter_ready == 1 && gGlobal.m_CAN.lifter_msg_result != 0)
	{
		gGlobal.m_stack.operationDoorAcc.stepper_des_prev = gGlobal.m_stack.operationDoorAcc.stepper_destination ;
		gGlobal.m_stack.operationDoorAcc.stepper_destination = 0;
		gGlobal.m_MOTORLifter.dir = 0;
		gGlobal.m_stack.operationDoorAcc.stepper_distance =0;
		gGlobal.m_stack.operationAccTimeforLifter =0;
		gGlobal.m_MOTORLifter.status = BRAKE;
		err[0] = gGlobal.m_MOTORLifter.err_flag ;
		AppResultRes(CMD_LOCKER_DOOR,gGlobal.m_CAN.lifter_msg_result , err, 1);
		gGlobal.m_CAN.lifter_ready = 0;
		gGlobal.m_CAN.lifter_msg_result = 0;
		gGlobal.m_stack.operationDoorAcc.stepper_pulse_cnt =0;
		
	}
		vTaskDelay(200);
	}

}

void locker_task(void * pvParameters)
{
	while(1)
	{
		if(EventGroupHandle != NULL)
		{
			xEventGroupWaitBits( 	(EventGroupHandle_t) 	EventGroupHandle,
														(EventBits_t			 ) 	LOCKER_EVENTBIT,
														(BaseType_t				 ) 	pdTRUE,
														(BaseType_t				 ) 	pdFALSE,
														(TickType_t				 ) 	portMAX_DELAY );			
		
			if (gGlobal.m_LOCKERUpper.status != 0)
			{
				switch(gGlobal.m_LOCKERUpper.status)
				{
					case CLOCKWISE:
						LockerRelease_Config(ID_01);
					break;
					
					case COUNTERCLOCKWISE:
						LockerLatch_Config(ID_01);
					break;
					
				}
				gGlobal.m_CAN.locker_msg_result = 1;
				if (gGlobal.m_LOCKERUpper.err_flag !=0)
					gGlobal.m_CAN.locker_msg_result = 2;
			}
			else if(gGlobal.m_LOCKERLower.status != 0)
			{
				switch(gGlobal.m_LOCKERLower.status)
				{
					case CLOCKWISE:
						LockerRelease_Config(ID_02);
					break;
					
					case COUNTERCLOCKWISE:
						LockerLatch_Config(ID_02);
					break;
				}
				gGlobal.m_CAN.locker_msg_result = 1;
				if (gGlobal.m_LOCKERLower.err_flag !=0)
					gGlobal.m_CAN.locker_msg_result = 2;				
			}
		}
		
		else
		{
			vTaskDelay(10);
		}
	
	
	}

}

void adc_processing_task(void * pvParameters)
{
	while(1)
	{
		if(BinarySemaphore_ADCDATA != NULL)
		{
			errcode = pdFALSE;
			errcode = xSemaphoreTake(BinarySemaphore_ADCDATA, portMAX_DELAY);
			if (errcode == pdTRUE){
				FetchingCurrentValue();
				num ++;
				if(gGlobal.m_CAN.locker_ready == 1 && DEBUG == 1 && gGlobal.m_CAN.door_ready == 0)
					printf("Warning: the number ADC conversion is %d",gGlobal.m_OCPData.currentCH4);
				if(gGlobal.m_CAN.door_ready == 1 && DEBUG == 1)
					printf("Current: upper channel current = %d mA, lower channel current = %d mA",gGlobal.m_OCPData.currentCH1, gGlobal.m_OCPData.currentCH3);
				if (gGlobal.m_CAN.lifter_ready == 1 && DEBUG == 1)
					printf("Current: lifter channel current = %d mA",gGlobal.m_OCPData.currentCH2);
				ADC_Cmd(ADC1, ENABLE);
				DMA_Cmd(DMA1_Channel1, ENABLE);	
				TIM_Cmd(TIM4, ENABLE);
			}
		}
		else if(errcode == pdFALSE)
			vTaskDelay(10);
	
	}

}

void usart_decode_task(void *pvParameters)
{
	BaseType_t err=pdFALSE;
	while(1)
	{
		if(BinarySemaphore_USART != NULL)
		{
			err = xSemaphoreTake(BinarySemaphore_USART, portMAX_DELAY);
			if(err == pdTRUE)
				UsartFun();
		}
		
		else if(err==pdFALSE)
		{
			vTaskDelay(10);
		}
	
	}


}
void PWRInit(void)
{
		GPIO_InitTypeDef  GPIO_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);	
		PWRSwitch = 0;
}

void InitGlobalSet(void)
{
    BYTE i=0,j=0;
    //以下为系统名称 10字节
    const BYTE sys_model_name[10] = "RUN3WAISHE";
    //以下为系统序列号 16字节
    const BYTE sys_serial_number[16] = "abcdefghij123456";
    //BYTE btpasswd[UPDATE_PASSWD_LENGTH]="UPDATE";
    const BYTE btpasswd[UPDATE_PASSWD_LENGTH]={'U','P','D','A','T','E'};   
    for (i=0;i<MAX_COM_INDEX;i++)
    {
        gGlobal.m_ComSet[i].m_PushIndex = 0;
        gGlobal.m_ComSet[i].m_btRecvState = 0;
        for (j=0;j<COM_CACHE_PACK_COUNT;j++)
        {
            gGlobal.m_ComSet[i].m_RecvPack[j].m_Pos = 0;
            gGlobal.m_ComSet[i].m_RecvPack[j].m_Len = 0;
        }
    }

    gGlobal.m_LastActiveComType = 0;
    gGlobal.m_LocalTime = 0;
    for(i=0; i<UPDATE_PASSWD_LENGTH; i++)
    {
        gGlobal.m_btPWD[i] = btpasswd[i];
    }
    //DeviceInfo
    for(i=0; i<10; i++)
    {
        gGlobal.m_DeviceInfo.model_name[i] = sys_model_name[i];
    }   
    gGlobal.m_DeviceInfo.firmware_version_H8bit = SYS_SW_VERSION_H8bit;
    gGlobal.m_DeviceInfo.firnware_version_L8bit = SYS_SW_VERSION_L8bit;
    gGlobal.m_DeviceInfo.hardware_version_H8bit = SYS_HW_VERSION_H8bit;
    gGlobal.m_DeviceInfo.hardware_version_L8bit = SYS_HW_VERSION_L8bit;
    for(i=0; i<16; i++)
    {
        gGlobal.m_DeviceInfo.serial_number[i] = sys_serial_number[i];
    }
}

void OCP0_IRQHandler(void)
{
	if (EXTI_GetFlagStatus(MOTORUPPER_INT_EXTI_LINE) == SET)
	{
		EXTI_ClearITPendingBit(MOTORUPPER_INT_EXTI_LINE);
		EXTI_ClearFlag(MOTORUPPER_INT_EXTI_LINE);
		if (UPPERMOTOR_OCP_IN == RESET && gGlobal.m_CAN.door_ready ==1)
		{
			DoorSingleMode_Running(BRAKE, 0,1) ;
			gGlobal.m_Status.Upperdoor_OCPFLAG = 1;
			printf("Upperdoor OCP is triggerred\r\n");
		}
	}
}

void LifterMidder_IRQHandler(void)
{
	if (EXTI_GetFlagStatus(LIFTER_TOP_INT_EXTI_LINE) == SET)
	{
		/* code */
		EXTI_ClearFlag(LIFTER_TOP_INT_EXTI_LINE);
		EXTI_ClearITPendingBit(LIFTER_TOP_INT_EXTI_LINE);
// IRQ Trigger of middle position for lifter
		if(gGlobal.m_MOTORLifter.status == RUNNING)
		{
			if (LIFTER_TOP_IN)
			{
				/* code */
				LIFTERTIMERDISABLE;
				TIM_ITConfig(TIM1,TIM_IT_CC2,DISABLE ); 
				if ( gGlobal.m_MOTORLifter.dir == COUNTERCLOCKWISE)
				{
					/* code */
					gGlobal.m_stack.lifterEndingSuccess = 1;
				}
			}		
		}
	}	
}

void Upperdoor_IRQHandler(void)
{
	if (EXTI_GetFlagStatus(UPPERDOOR_OPEN_INT_EXTI_LINE) == SET)
	{
		/* UPPERDOOR OPEN EXTI LINE */
		EXTI_ClearITPendingBit(UPPERDOOR_OPEN_INT_EXTI_LINE);
		EXTI_ClearFlag(UPPERDOOR_OPEN_INT_EXTI_LINE);	
		if (UPPERDOOR_OPEN_IN )
		{		
			DoorSingleMode_Running(BRAKE, 0,1) ;
			if (gGlobal.m_CAN.door_ready ==1 && gGlobal.m_stack.operationDIR == CLOCKWISE)
			{
				/* code */
				gGlobal.m_stack.doorEndingSuccess++;
			}
			else if (DEBUG == 1)
				printf("Warning : upperdoor-opened sensor problem!\r\n");
		}		
	}
}

void OCP4_IRQHandler(void)
{
	if(EXTI_GetFlagStatus(MOTORLOWER_INT_EXTI_LINE) == SET)
	{
		EXTI_ClearITPendingBit(MOTORLOWER_INT_EXTI_LINE);
		EXTI_ClearFlag(MOTORLOWER_INT_EXTI_LINE);	
		if( LOWERMOTOR_OCP_IN == RESET)
		{
			;
		}
	}
}

void OCP9_5_IRQHandler(void)
{

	if(EXTI_GetFlagStatus(LIFTER_INT_EXTI_LINE) == SET)
	{
		EXTI_ClearITPendingBit(LIFTER_INT_EXTI_LINE);
		EXTI_ClearFlag(LIFTER_INT_EXTI_LINE);	
		
		if( LIFTERMOTOR_OCP_IN == RESET)
		{
			gGlobal.m_MOTORLifter.err_OCPcnt ++;
			if (gGlobal.m_MOTORLifter.err_OCPcnt >=2)
			{
				/* code */
				Lifter_Running(IDLE, 0);
				gGlobal.m_Status.Lifter_OCPFLAG = 1;
				gGlobal.m_Status.Lifter_OCPCNT ++;
				gGlobal.m_MOTORLifter.err_OCPcnt =0;
			}
		}
		if (LOWERDOOR_OPEN_IN)
		{
			/* code */
			DoorSingleMode_Running(BRAKE, 0,2) ;
			if (gGlobal.m_CAN.door_ready ==1 && gGlobal.m_stack.operationDIR == CLOCKWISE)
			{
				/* code */
				gGlobal.m_stack.doorEndingSuccess++;
			}
			else if (DEBUG == 1)
				printf("Warning : lowerdoor-opened sensor problem!\r\n");
		}
	}
	
	if (EXTI_GetFlagStatus(LOWERDOOR_CLOSE_INT_EXTI_LINE) == SET)
	{
		/* code */
		EXTI_ClearFlag(LOWERDOOR_CLOSE_INT_EXTI_LINE);
		EXTI_ClearITPendingBit(LOWERDOOR_CLOSE_INT_EXTI_LINE);
		if (LOWERDOOR_CLOSE_IN)
		{
			/* code */
			DoorSingleMode_Running(BRAKE, 0,2) ;
			if (gGlobal.m_CAN.door_ready ==1 && gGlobal.m_stack.operationDIR == COUNTERCLOCKWISE )
			{
				/* code */
				gGlobal.m_stack.doorEndingSuccess++;
			}
			else if (DEBUG == 1)
				printf("BUG : upperdoor-closed sensor problem!\r\n");
		}
	}
}

void OCP15_10_IRQHandler(void)
{
	if(EXTI_GetFlagStatus(LOCKER_INT_EXTI_LINE) == SET)
	{
		EXTI_ClearITPendingBit(LOCKER_INT_EXTI_LINE);
		EXTI_ClearFlag(LOCKER_INT_EXTI_LINE);	
		if( LOCKERMOTOR_OCP_IN == RESET)
		{
			if ( gGlobal.m_LOCKERUpper.status != 0 && gGlobal.m_LOCKERLower.status == 0)
			{
				/* code */
				gGlobal.m_LOCKERUpper.err_OCPcnt ++;
				if (gGlobal.m_LOCKERUpper.err_OCPcnt >=2)
				{
					/* code */
					gGlobal.m_LOCKERUpper.err_OCPcnt = 0; 
					gGlobal.m_Status.LOCKER_OCPFLAG = 1;
					gGlobal.m_Status.LOCKER_ErrCNT ++;
				}
			}
		}
	}

	if (EXTI_GetFlagStatus(UPPERDOOR_CLOSE_INT_EXTI_LINE) == SET)
	{
		/* code */
		EXTI_ClearITPendingBit(UPPERDOOR_CLOSE_INT_EXTI_LINE);
		EXTI_ClearFlag(UPPERDOOR_CLOSE_INT_EXTI_LINE);
		if (UPPERDOOR_CLOSE_IN )
		{
			DoorSingleMode_Running(BRAKE, 0,1) ;
			if (gGlobal.m_CAN.door_ready ==1 && gGlobal.m_stack.operationDIR == COUNTERCLOCKWISE )
			{
				/* code */
				gGlobal.m_stack.doorEndingSuccess++;
			}
			else if (DEBUG == 1)
				printf("BUG : upperdoor-closed sensor problem!\r\n");
		}	
	}
// IRQ Trigger of bottom position for lifter
	if (EXTI_GetFlagStatus(LIFTER_BOTTOM_INT_EXTI_LINE) == SET)
	{
		/* code */
		EXTI_ClearFlag(LIFTER_BOTTOM_INT_EXTI_LINE);
		EXTI_ClearITPendingBit(LIFTER_BOTTOM_INT_EXTI_LINE);
		if(gGlobal.m_MOTORLifter.status == RUNNING)
		{
			if (LIFTER_BOTTOM_IN)
			{
				/* code */
				LIFTERTIMERDISABLE;
				TIM_ITConfig(TIM1,TIM_IT_CC2,DISABLE ); 
				if ( gGlobal.m_MOTORLifter.dir == CLOCKWISE)
				{
					/* code */
					gGlobal.m_stack.lifterEndingSuccess = 1;
				}
			}		
		}
	}
}


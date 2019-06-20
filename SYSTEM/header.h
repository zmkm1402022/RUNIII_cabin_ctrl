#ifndef __HEADER_H
#define __HEADER_H
#include <stdio.h>
#include "stm32f10x.h"
#include "usart.h"
#include "led.h"
#include "can.h"
#include "OCPSampling.h"
#include "timerapp.h"
#include "ds18b20.h"
#include "led.h"
#include "exDCMotor.h"
#include "stm32f10x_tim.h"
#include "delay.h"

/********************************************************************************************************************************/

#define DEBUG 			0
//extern EventGroupHandle_t EventGroupHandle; 

#define LOCKER_EVENTBIT (1<<0)
#define DOOR_EVENTBIT		(1<<1)
#define LIFTER_EVENTBIT		(1<<2)
#define FAULTTRIG_EVENTBIT (1<<3)
#define LED_EVENTBIT   (1<<4)
//define the firmware version as v1.0
#define 	SYS_SW_VERSION_H8bit          1
#define 	SYS_SW_VERSION_L8bit          0
//define the hardware version as v1.2
#define 	SYS_HW_VERSION_H8bit          1
#define 	SYS_HW_VERSION_L8bit          2

#define CLOCKWISE 											0x11  // 1) door: clockwise = open the door; 2) lifter: clockwise = go to the bottom; 3) locker: clockwise = unlock
#define COUNTERCLOCKWISE 								0x12	 // 1) door: CCW = close the door; 2) lifter: CCW = go to the middle level; 3) locker: CCW = lock
#define IDLE 														0x13
#define BRAKE														0x14
#define RUNNING  												0x15
#define ID_01														1
#define ID_02														2
#define ID_03														3


#define BUSVOLTAGE											ADC_Channel_5
#define MOTOROCP_01 										ADC_Channel_6
#define MOTOROCP_02 										ADC_Channel_7
#define MOTOROCP_03 										ADC_Channel_14
#define MOTOROCP_04 										ADC_Channel_15

#define NumberofChannel 							5
#define NumberofCycle 								1

#define IRQ_ADC_Priority 							10
#define IRQ_EXTI_Priority 						5

/*  Over current protection ports */
#define MOTORUPPER_INT_EXTI_LINE				EXTI_Line0
#define MOTORLOWER_INT_EXTI_LINE				EXTI_Line4
#define LIFTER_INT_EXTI_LINE						EXTI_Line9
#define LOCKER_INT_EXTI_LINE						EXTI_Line14
/*  switching sensor ports*/
#define LIFTER_TOP_INT_EXTI_LINE 				EXTI_Line2
#define LIFTER_BOTTOM_INT_EXTI_LINE 		EXTI_Line10
#define UPPERDOOR_CLOSE_INT_EXTI_LINE 	EXTI_Line13
#define UPPERDOOR_OPEN_INT_EXTI_LINE 		EXTI_Line1
#define LOWERDOOR_CLOSE_INT_EXTI_LINE 	EXTI_Line8
#define LOWERDOOR_OPEN_INT_EXTI_LINE 		EXTI_Line9





#define ADCCHANNEL_IRQHandler							DMA1_Channel1_IRQHandler
#define OCP0_IRQHandler										EXTI0_IRQHandler
#define OCP4_IRQHandler										EXTI4_IRQHandler
#define OCP9_5_IRQHandler									EXTI9_5_IRQHandler
#define OCP15_10_IRQHandler								EXTI15_10_IRQHandler
#define Upperdoor_IRQHandler							EXTI1_IRQHandler
#define LifterMidder_IRQHandler						EXTI2_IRQHandler  

/* EXTI-LINE input state*/
#define LIFTER_TOP_IN 										GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2)		// PD2: switch sensor at middle position for lifter  (CONN: J25)
#define LIFTER_BOTTOM_IN 									GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_10)	// PA10: switch sensor at bottom position for lifter (CONN: J25)

#define UPPERDOOR_CLOSE_IN								GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)	// PB13: switch sensor at closed position for upperdoor (CONN: J19)
#define UPPERDOOR_OPEN_IN									GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)  //PB1 : switch sensor at opened position for upperdoor (CONN: J19)

#define LOWERDOOR_CLOSE_IN								GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8)	//PA8: switch sensor at closed position for upperdoor (CONN: J21)
#define LOWERDOOR_OPEN_IN									GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_9)	//PB9 : switch sensor at opened position for upperdoor (CONN: J21)



/* fetch the input state of locker switches*/
#define UPPERLOCKER_RELEASED_IN						GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_10)
#define UPPERLOCKER_LOCKED_IN							GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12)

#define LOWERLOCKER_RELEASED_IN						GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_11)
#define LOWERLOCKER_LOCKED_IN							GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)

#define LIFTERLOCKER_RELEASED_IN					GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_15)
#define LIFTERLOCKER_LOCKED_IN						GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3)


/* OCP Function*/
#define UPPERMOTOR_OCP_IN									GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)		
#define LOWERMOTOR_OCP_IN									GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)	
#define LIFTERMOTOR_OCP_IN								GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9)		
#define LOCKERMOTOR_OCP_IN								GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)		

/*RESET Function*/
#define GetValue_NJST_Pin									GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_6)		

/*usart buffer size*/
#define PACK_SIZE    									180+1
#define 	COM_CACHE_PACK_COUNT    		2		//各个通信接口的数据结构
#define 	MAX_COM_INDEX 					2		//通信接口个数，包括 串口1、串口2
//////////////////////////////////////////////////////////////////////////////
         
//协议数据包方面的宏定义       
#define CMD_BAG_HEADER_BYTE1             0xAB        //协议包头
#define CMD_BAG_HEADER_BYTE2             0x56        //协议包头

#define CMD_BAG_FLAG_RESULT							 0x02				
#define CMD_BAG_FLAG_BYTE                0x01       //协议标志位 外设板-->安卓 response 
#define CMD_BAG_FLAG_ANZHUO2WAISHE			 0x00	
//升级命令字
#define CMD_START_UPDATA                0x15      //开始升级命令
#define CMD_UPDATA_BAG                  0x16      //更新数据包命令
#define CMD_END_UPDATA                  0x17      //结束升级命令

//通用命令字
#define CMD_PING_TEST                   0x01      //ping帧测试命令
#define CMD_READ_DEVICEINFOR            0x02      //读取设备型号命令
#define CMD_RESET_DEVICE                 0x10      //复位命令
#define CMD_START_RESPOND               0x11      //启动自助反馈命令
#define CMD_END_RESPOND                 0x12      //停止自助反馈命令

// lifter operation
#define CMD_MOTOR_LIFTER							0x2B

// upper locker operation
#define CMD_LOCKER_UPPERLOWER							0x29
// door operation
#define CMD_LOCKER_DOOR										0x2A

// state check (door/locker/lifter positions)
#define CMD_POSITION_CHECK							0x03

#define CMD_LED_CTRL								0x2C
#define CMD_POS_MAINPART						0x03

//错误类型
#define ERR_CMD                         0x8000      //命令不支持
#define ERR_CMD_DATAERROR								0x8003      // 命令内容不支持
#define ERR_FORMAT                      0x8001      //格式错误
#define ERR_OPERATION                   0x8002      //操作错误
#define ERR_CMD_ERROR_BUSY							0x8004			//同类动作正在执行
#define ERR_XOR													0x8006
#define ERR_LENGTH                      0x05      //总长度
#define ERR_LEN                         0x02      //数据长度
//其他宏定义
#define   UPDATE_PASSWD_LENGTH            6 		//升级口令长度

//读写器存储位置
#define BOOT_ADDRESS            			(uint32_t)0x08000000
#define REMAIN_ADDRESS          			(uint32_t)0x08004000
#define BOOT_PARAM_ADDRESS_UART      (uint32_t)0x08004800
#define BOOT_LAST_ADDRESS       			(uint32_t)0x08004FFF

#define PROCESS_ADDRESS         (uint32_t)0x08005000
#define PROCESS_LAST_ADDRESS    (uint32_t)0x0800FFFF  

#define PARAM_ADDRESS           (uint32_t)0x0800F800  

#define CHIP_LAST_ADDRESS   	(uint32_t)0x0800FFFF 

typedef enum type_name
{
	LOCKER_UP = 1,
	LOCKER_DOWN,
	LOCKER_DUAL
} MODE;

typedef struct
{
	BYTE	m_pBuffer[PACK_SIZE + 1];
	BYTE  m_Flag;
	BYTE	XorVal;
	WORD	m_Pos;
	WORD	m_Len;
} All_PACK;

//接口通信类型
typedef enum{
    COMM_TYPE_COM1 = 0,
    COMM_TYPE_COM2 = 1,
}COMMTYPE;

//通信接口结构体
typedef struct 
{
    BYTE                m_btRecvState;                                  //当前解析的状态机
    All_PACK            m_RecvPack[COM_CACHE_PACK_COUNT];               //解析后数据包的缓存。
    BYTE                m_PushIndex;                                    //当前push数据包的索引
}ComSet;
/**********************Threshold Parameters for Protection*************************/
typedef struct{
	uint8_t 		status; // default = 0 , CW/CCW/IDLE/BRAKE 
	uint16_t		err_flag; 	// no error = 0x00, overcurrent = 0x01, undervoltage = 0x02, timeout = 0x03   
	int16_t			interval;
	uint32_t  	timetostart;
	int16_t 	 	dutycycle;
	uint8_t   	dir;
	uint8_t 		err_OCPcnt;
	uint16_t		threshold;
} MOTORPARAM;


typedef struct{
	uint8_t 	ID; //  1 = upper cabin led; 2 = lower cabin led; 3 = upper/lower cabin leds ; 10 = outer left led ; 11 = outer right led
	uint8_t   mode; 
	uint8_t 	status;  // on/off
	uint16_t	dutycycle; //
} LEDPARAM;

typedef struct{
	uint16_t rawdata[NumberofChannel*NumberofCycle];
	uint16_t voltbus;
	uint16_t currentCH1;
	uint16_t currentCH2;
	uint16_t currentCH3;
	uint16_t currentCH4;
} CURRENTVECTOR;



typedef struct
{
    BYTE 	model_name[10];
    BYTE 	firmware_version_H8bit;
    BYTE 	firnware_version_L8bit;
    BYTE 	hardware_version_H8bit;
    BYTE 	hardware_version_L8bit;
    BYTE 	serial_number[16];
}DeviceInfo;

typedef struct{
	uint8_t 	NJRSTFlag;
	uint8_t 	BROADCASTFlag;       			// 0x01 = broadcast; 0x02 = stop broadcasting
	uint8_t 	UPPERLOCKER_STATUS; 		  //0X01 = locked ; 0x02 = unlocked ; 0x03 = in-between
	uint8_t 	LOWERLOCKER_STATUS;			  //0X01 = locked ; 0x02 = unlocked; 0x03 = in-between
	uint8_t 	LIFTER_STATUS;     				//0X01 = middle position ; 0x02 = bottom position;0x03 = in-between
	uint8_t 	UPPERDOOR_STATUS;  				//0X01 = open state ; 0x02 = closed state;0x03 = in-between
	uint8_t 	LOWERDOOR_STATUS;					//0X01 = open state ; 0x02 = closed state; 0x03 = in-between
//	uint8_t 	Electric_LockerChannel; 	// 0 = on; 1 = off;
	uint16_t  UPPERDOOR_ErrCNT;   			// _ErrCNT is used to record the OCP/error situation permantantly
	uint8_t   Upperdoor_OCPFLAG;
	uint8_t		Upperdoor_OCPErr_CNT;
	uint8_t   Lowerdoor_OCPFLAG;
	uint8_t		Lowerdoor_OCPErr_CNT;
	uint16_t  LOWERDOOR_ErrCNT;   // _ErrCNT is used to record the OCP/error situation permantantly
	uint16_t  LIFTER_ErrCNT;
	uint16_t  LOCKER_ErrCNT;   // _ErrCNT is used to record the OCP/error situation permantantly
	uint8_t   LOCKER_RECOVERY_CNT; 
	uint16_t  Lifter_OCPCNT;
	uint8_t   Lifter_OCPFLAG;
	uint8_t   LOCKER_OCPFLAG;
	uint8_t   LOCKER_Freeze;
} MOTIONFLAG;


typedef struct{
	uint8_t operation_mode;  
	uint8_t door_ready;     // 1 = identify the door task is running; 0 = the door task is idle
	uint8_t locker_msg_result;  // 1 = task succeed; 2 = task fail; 0= default;
	uint8_t door_msg_result; 
	uint8_t lifter_msg_result;
	uint8_t locker_ready;
	uint8_t locker_msg;   // store the received cmd for lockers. not used in usart 
	uint8_t lifter_ready;
} COMMUNICATION;


typedef struct
{
	uint16_t accTime;    
	uint16_t deaccTime;  
	uint8_t  accStep;   
	uint8_t  deaccStep;    
	uint32_t realtimeIntCnt;  
	uint32_t stepper_distance;
	uint32_t stepper_pulse_cnt;  
	u8			 stepper_destination;
	u32			 stepper_time_cnt;
	u8 			 stepper_des_prev;
	
} ACCELERATIONPARAM;

typedef struct 
{
	uint8_t 					operationID;  // 1= uppermotor, 2 = lowermotor, 3 = dualmotor
	uint8_t 					operationDIR; //  0x11 = CW, 0x12 = CCW
	ACCELERATIONPARAM operationDoorAcc;
	int16_t 					operationDUTY;  
	uint16_t 					operationAccTimeforLifter;
	uint16_t					operationSpeedforLifter;
	uint16_t 					operationDeaccTimeforLifter;
	uint16_t 					operationIntCNTforLifter;
	uint32_t 					temperature_interval;
	uint32_t 					boradcast_interval;
	uint16_t					door_interval;
	uint32_t 					door_timetostart;
	uint8_t					  doorEndingSuccess;
	uint8_t  					lifterEndingSuccess;
	uint16_t					StepperAccSpeed;
	uint16_t					StepperAccDeltaAT;
} OPERATIONSTACK;

typedef struct
{
	u8								p_buffer[50];
	u8								p_flag;
	u8								p_length;
	u8								p_broadcastflag;
} USARTPARAM;


typedef struct
{
	DeviceInfo          m_DeviceInfo;
	volatile uint32_t		m_LocalTime;
	uint8_t							m_ADCstatus;
	uint16_t 						m_Errcode;
	MOTORPARAM					m_MOTORUpper;
	MOTORPARAM					m_MOTORLower;
	MOTORPARAM					m_MOTORLifter;
	MOTORPARAM					m_LOCKERUpper;
	MOTORPARAM					m_LOCKERLower;
	CURRENTVECTOR 			m_OCPData;
	LEDPARAM						m_LED;
	COMMUNICATION				m_CAN;
	MOTIONFLAG					m_Status;
	OPERATIONSTACK			m_stack;
	short 						temperature;
	USARTPARAM					m_UsartData;
	u8									m_LastActiveComType;
	u8									m_AppDataLen;
	u8									m_btPWD[UPDATE_PASSWD_LENGTH];
	ComSet             m_ComSet[MAX_COM_INDEX];
}GlobalSet;

extern GlobalSet gGlobal;


extern BOOL Check1MSTick(DWORD dwCurTickCount, DWORD dwStart, DWORD DelayTime);




/*******************************************************************低字节在前*************************************************************************/
#define InitPack(pack)	\
	(pack).m_Pos = 0;	\
	(pack).m_Len = 0;	\
	(pack).m_pBuffer[0] = 0;

#define BYTEToPack(pack,btVal)\
	if ((pack).m_Pos + 1 < PACK_SIZE)	\
	{\
		(pack).m_pBuffer[(pack).m_Pos++] = btVal;\
	}\
	(pack).m_Len = (pack).m_Pos;

#define WORDToPack(pack,wVal) \
	if ((pack).m_Pos + 2 < PACK_SIZE) \
	{\
		(pack).m_pBuffer[(pack).m_Pos++] = (BYTE)(wVal&0x00ff);\
		(pack).m_pBuffer[(pack).m_Pos++] = (BYTE)((wVal>>8)&0x00ff);\
	}\
	(pack).m_Len = (pack).m_Pos;


#define DWORDToPack(pack,wVal) \
	if ((pack).m_Pos + 4 < PACK_SIZE) \
	{\
		(pack).m_pBuffer[(pack).m_Pos++] = (BYTE)(wVal&0x00ff);\
                (pack).m_pBuffer[(pack).m_Pos++] = (BYTE)((wVal>>8)&0x00ff);\
                (pack).m_pBuffer[(pack).m_Pos++] = (BYTE)((wVal>>16)&0x00ff);\
                (pack).m_pBuffer[(pack).m_Pos++] = (BYTE)((wVal>>24)&0x00ff);\
	}\
	(pack).m_Len = (pack).m_Pos;

#define BYTEFromPack(pack,btVal)\
	if ((pack).m_Pos + 1 <= (pack).m_Len) \
	{\
		(btVal) = (pack).m_pBuffer[(pack).m_Pos++];\
	}\
	else\
	{\
		(btVal) = 0;\
	}
#define WORDFromPack(pack,wVal)\
	if ((pack).m_Pos + 2 <= (pack).m_Len)\
	{\
		(wVal) = (pack).m_pBuffer[(pack).m_Pos++];\
                (wVal) += (((WORD)(pack).m_pBuffer[(pack).m_Pos++]) << 8);\
	}


#define DWORDFromPack(pack,wVal)\
	if ((pack).m_Pos + 4 <= (pack).m_Len)\
	{\
                (wVal) =(pack).m_pBuffer[(pack).m_Pos++];\
		(wVal) +=(pack).m_pBuffer[(pack).m_Pos++]<<8;\
                (wVal) +=(pack).m_pBuffer[(pack).m_Pos++]<<16;\
                (wVal) += (pack).m_pBuffer[(pack).m_Pos++]<<24;\
	}

#define PUSHPackHeader(pack) \
	(pack).m_Pos = 0;	\
	(pack).m_Len = 0;	\
        BYTEToPack(pack,0x7F);\
        BYTEToPack(pack,0x7F);

#define PUSHPackTagHeader(pack) \
	(pack).m_Pos = 0;	\
	(pack).m_Len = 0;	\
        BYTEToPack(pack,CMD_BAG_HEADER_BYTE1);\
        BYTEToPack(pack,CMD_BAG_HEADER_BYTE2);

#endif

	

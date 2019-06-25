#include "sys.h"
#include "usart.h"	  
#include <stdio.h>
#include <string.h>
#include "header.h"
#include "FreeRTOS.h"				
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
u8 cmd;
u8 USART_rx_buf[150], USART_tx_buf[150];
static u8 tx_buf[5];
extern SemaphoreHandle_t BinarySemaphore_USART;
extern EventGroupHandle_t EventGroupHandle; 
void USART1_DMA_Config(void);
//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if UART1_PRINTF
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 

void _ttywrch(int ch)
{
ch = ch;
}
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

 

//����1�жϷ������


void uart_init(u32 bound){

  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOB| RCC_APB2Periph_AFIO, ENABLE);	
	GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOB, &GPIO_InitStructure);
   
  //USART2_RX	  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOA. 

   //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
  USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);//�������ڽ����ж�
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	USART1_DMA_Config();
	
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
  USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���
	
}


void USART1_DMA_Config(void)  
{  
  DMA_InitTypeDef DMA_InitStructure;  
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	
	DMA_DeInit(DMA1_Channel5); 
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR; ;  //DMA�������ַ
	DMA_InitStructure.DMA_MemoryBaseAddr =(uint32_t) gGlobal.m_UsartData.p_buffer;  //DMA�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //���ݴ��䷽�򣬴����贫���ڴ�
	DMA_InitStructure.DMA_BufferSize = PACK_SIZE;  //DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //�����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //����������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMAͨ�� xӵ�������ȼ� 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMA1_Channel5, &DMA_InitStructure);  //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��USART1_Tx_DMA_Channel����ʶ�ļĴ���	
	DMA_Cmd(DMA1_Channel5, ENABLE);
} 


uint8_t USART1_ComSend(u8 *pBuffer,uint32_t btLen)
{
    int i;
    for (i=0;i<btLen;i++)
    {
      USART_SendData(USART1, pBuffer[i]);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
			{}
//			USART_ClearFlag(USART1,USART_FLAG_TC);
    }
    return btLen;
}





BYTE CalcXORSum(u8 *pData, WORD wDataLen)
{
    BYTE i=0,ResuValue=0;
    for(; i<wDataLen; i++)
    {
       ResuValue ^= pData[i];
    }
    return ResuValue;
}


/*****************************************************CheckPWD<PWD>����У��********************************************************/
BYTE CheckPWD(BYTE *btPWD)
{
    BYTE i=0;
    for(; i<UPDATE_PASSWD_LENGTH; i++)
    {
        if(gGlobal.m_btPWD[i] != btPWD[i])
            return FALSE;
    }
    return TRUE;
}


/*Ӧ������ķ��ͺ���*/
void SendUp(All_PACK *pPack)
{
    switch(gGlobal.m_LastActiveComType)
    {
        case COMM_TYPE_COM2:
//           USART2_ComSend(pPack->m_pBuffer,pPack->m_Len);
            break;
        case COMM_TYPE_COM1:
            USART1_ComSend(pPack->m_pBuffer,pPack->m_Len);
            break;
        default:
            break;            
    }
}


void SendErrCode(WORD wdErrType, u8 cmd)
{
		All_PACK ErrSendPack;
    InitPack(ErrSendPack);
    PUSHPackTagHeader(ErrSendPack);  // push header into the pack
    BYTEToPack(ErrSendPack,ERR_LENGTH);
    BYTEToPack(ErrSendPack,0x01);    //Flag
    BYTEToPack(ErrSendPack,ERR_LEN); //data length
    BYTEToPack(ErrSendPack,cmd);  // cmd
    WORDToPack(ErrSendPack,wdErrType);
    BYTEToPack(ErrSendPack,CalcXORSum(&ErrSendPack.m_pBuffer[2], ErrSendPack.m_Pos-2));//У���ֽ� ǰ����һ����ͷ0xAA 0x55�ֽڲ���Ϊ�������
    SendUp(&ErrSendPack);
}
/* ͨ�õ���ȷӦ�� */
void AppCommonRightRes()
{
	All_PACK SendPack;
	//������Ӧ������
    InitPack(SendPack);//��ʼ��
    PUSHPackTagHeader(SendPack);//��ͷ�ֽ�0xAA 0x55ѹջ
    BYTEToPack(SendPack,0x03);//�ܳ���ѹջ Length
    BYTEToPack(SendPack,CMD_BAG_FLAG_BYTE);//Flag�ֽ�
    BYTEToPack(SendPack,0x00);//���ݳ����ֽ� Len
    BYTEToPack(SendPack,0x01);//cmd
    BYTEToPack(SendPack,CalcXORSum(&SendPack.m_pBuffer[2], SendPack.m_Pos-2));//У���ֽ� ǰ����һ����ͷ0xAA 0x55�ֽڲ���Ϊ�������
    SendUp(&SendPack);
}

/* operation  feedback */
void AppOperationRes(u8 rcmd)
{
	All_PACK SendPack;
	//������Ӧ������
    InitPack(SendPack);//��ʼ��
    PUSHPackTagHeader(SendPack);//��ͷ�ֽ�0xAA 0x55ѹջ
    BYTEToPack(SendPack,0x03);//�ܳ���ѹջ Length
    BYTEToPack(SendPack,CMD_BAG_FLAG_BYTE);//Flag�ֽ� 0x01
    BYTEToPack(SendPack,0x00);//���ݳ����ֽ� Len
    BYTEToPack(SendPack,rcmd);//cmd
    BYTEToPack(SendPack,CalcXORSum(&SendPack.m_pBuffer[2], SendPack.m_Pos-2));//У���ֽ� ǰ����һ����ͷ0xAA 0x55�ֽڲ���Ϊ�������
    SendUp(&SendPack);
}
/* operation result feedback 
[input] result: 0 = ignore this byte, xx = operation result
[input] err: array for sending datas
[input] cnt: 0 = ignore the Data section
*/
void AppResultRes(u8 rcmd, u8 result, u8 err[], u8 cnt)
{
	All_PACK SendPack;
	u8 i;
	//������Ӧ������
	InitPack(SendPack);//��ʼ��
	PUSHPackTagHeader(SendPack);//��ͷ�ֽ�0xAA 0x55ѹջ
	
	if(result !=0){
		BYTEToPack(SendPack,cnt+4);//�ܳ���ѹջ Length
	}
	else {
		BYTEToPack(SendPack,cnt+3);//�ܳ���ѹջ Length
	}
	
	
	
	BYTEToPack(SendPack,CMD_BAG_FLAG_RESULT);//Flag�ֽ�
	
	if(result >0){
		BYTEToPack(SendPack,cnt+1);//���ݳ����ֽ� Len
	}
	else{
		BYTEToPack(SendPack,cnt);
	}
	
	BYTEToPack(SendPack,rcmd);//cmd
	
	if(result > 0){
		BYTEToPack(SendPack,result);
	}
	if(cnt != 0)
	{
		for(i=0; i<cnt; i++){
			BYTEToPack(SendPack,err[i]);	
		}
	}
	BYTEToPack(SendPack,CalcXORSum(&SendPack.m_pBuffer[2], SendPack.m_Pos-2));//У���ֽ� ǰ����һ����ͷ0xAA 0x55�ֽڲ���Ϊ�������
	SendUp(&SendPack);
}

/* ping֡�������� */
void AppPingTest(All_PACK *pack)
{
    if(gGlobal.m_AppDataLen != 0)  // if the length of data (in payload) is not zero
    {
        SendErrCode(ERR_FORMAT, 0x01);
        return;
    }
	AppCommonRightRes();
}
/* ��ȡ�豸�ͺ����� */
void AppReadDeviceinfor(All_PACK *pack)
{
    BYTE i,*DeviceInformation;
    All_PACK SendPack;
    if(gGlobal.m_AppDataLen != 0)
    {
        SendErrCode(ERR_FORMAT, 0x02);
        return;
    }
    //������Ӧ������
    InitPack(SendPack);//��ʼ��
    PUSHPackTagHeader(SendPack);//��ͷ�ֽ�0xAA 0x55ѹջ
    BYTEToPack(SendPack,0x21);//�ܳ���ѹջ Length
    BYTEToPack(SendPack,CMD_BAG_FLAG_BYTE);//Flag�ֽ�
    BYTEToPack(SendPack,0x1E);//���ݳ����ֽ� Len
    BYTEToPack(SendPack,0x01);//�ɹ���־�ֽ�
    DeviceInformation = (BYTE *)&gGlobal.m_DeviceInfo; 
    for(i=0; i< sizeof(gGlobal.m_DeviceInfo); i++)
    {
        BYTEToPack(SendPack,DeviceInformation[i]);//�ɹ���־�ֽ�
    }
    BYTEToPack(SendPack,CalcXORSum(&SendPack.m_pBuffer[2], SendPack.m_Pos-2));//У���ֽ� ǰ����һ����ͷ0xAA 0x55�ֽڲ���Ϊ�������
    SendUp(&SendPack);
}
/* Soft reset ��λ���� */
void AppRestDevice(All_PACK *pack)
{
    if(gGlobal.m_AppDataLen != 0)
    {
        SendErrCode(ERR_FORMAT, 0x10);
        return;
    }
		TIM_Cmd(TIM5, DISABLE); 
}
/* ���������������� */
void AppStartRespond(All_PACK *pack)
{
    if(gGlobal.m_AppDataLen != 0)
    {
        SendErrCode(ERR_FORMAT, 0x11);
        return;
    }
	gGlobal.m_UsartData.p_broadcastflag = 1;
}
///* ֹͣ������������ */
void AppEndRespond(All_PACK *pack)
{
    if(gGlobal.m_AppDataLen != 0)
    {
        SendErrCode(ERR_FORMAT, 0x12);
        return;
    }
	gGlobal.m_UsartData.p_broadcastflag = 0;
}

/* ��ʼ�������� */ 
void AppStartUpdata(All_PACK *pack)
{
    BYTE i,boardIndex;
    DWORD dwStart;
		uint32_t JumpAddress=0;
    pFunction Jump_To_Application;
    BYTE btpasswd[UPDATE_PASSWD_LENGTH];
    if(gGlobal.m_AppDataLen != (UPDATE_PASSWD_LENGTH+1))
    {
        SendErrCode(ERR_FORMAT, 0x15);
        return;
    }
	BYTEFromPack(*pack,boardIndex);
    for(i=0; i<gGlobal.m_AppDataLen; i++)
    {
        BYTEFromPack(*pack,btpasswd[i]);
    }
    if(!CheckPWD(btpasswd))
    {
        SendErrCode(ERR_FORMAT, 0x15);
        return;
    }
	if(boardIndex == 0x01)
	{
		FLASH_Unlock();
		FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
		FLASH_ErasePage(BOOT_PARAM_ADDRESS_UART);
		FLASH_ProgramWord(BOOT_PARAM_ADDRESS_UART, 0x01);
		FLASH_Lock();
	}
    if(*(unsigned long *)BOOT_PARAM_ADDRESS_UART == 0x01)
    {    
		AppCommonRightRes();
        dwStart = gGlobal.m_LocalTime; //��ʱ
        while(Check1MSTick(gGlobal.m_LocalTime, dwStart, 5))//��ʱ,Ϊ���ڷ���������ȡʱ��
        {
        }
		//��תָ��
		JumpAddress = *(__IO uint32_t*) (BOOTCROCESS_ADDRESS + 4);
		/* Jump to user application */
		Jump_To_Application = (pFunction) JumpAddress;
		/* Initialize user application's Stack Pointer */
		__set_MSP(*(__IO uint32_t*) BOOTCROCESS_ADDRESS);
		Jump_To_Application();
    }
    else
    {
        SendErrCode(ERR_OPERATION, 0x15);
        return;
    }

}

u8 positioncheck(u8 pos_1, u8 pos_2)
{
	if(pos_1 == 1)
		return 1;
	else if(pos_2 == 1)
		return 2;
	else
		return 3;
}

void AppHandProtection(u8 * vault)
{
	u16 data[2];
	if(gGlobal.m_AppDataLen != 4)
	{
			SendErrCode(ERR_FORMAT, CMD_ADC_CALIBRA);
			return;
	}
	data[0] = vault[0]|(vault[1]<<8);
	data[1] = vault[2]|(vault[3]<<8);
	if(data[0]<THRESHOLD_MIN || data[1]<THRESHOLD_MIN)
	{
		SendErrCode(ERR_WRONG_DATA_RANGE, CMD_ADC_CALIBRA);
		return;
	}
	else if(data[0]>THRESHOLD_MAX || data[1]>THRESHOLD_MAX)
	{
		SendErrCode(ERR_WRONG_DATA_RANGE, CMD_ADC_CALIBRA);
		return;	
	}
	else
	{
		gGlobal.m_UpperDoorThreshold = data[0];
		gGlobal.m_LowerDoorThreshold = data[1];
		AppResultRes(CMD_ADC_CALIBRA, 0 , tx_buf, 0);
	}
}

void AppAdcCalibration(void)
{
	if(gGlobal.m_AppDataLen != 0)
	{
			SendErrCode(ERR_FORMAT, CMD_ADC_CALIBRA);
			return;
	}
	UPPERMOTORBRAKE;
	LOWERMOTORBRAKE;
	LOCKERSTANDBY;
	delay_xms(1000);	
	gGlobal.m_OCPData.cal_enable = 1;
	AppResultRes(CMD_ADC_CALIBRA, 0 , tx_buf, 0);
}


/*
return the current position of doors/lockers/ lifter
*/
void AppPosCheck(void )
{
	
	u8 posL, posR;
	if(gGlobal.m_AppDataLen != 0)
	{
			SendErrCode(ERR_FORMAT, CMD_POS_MAINPART);
			return;
	}
	posL = 	UPPERLOCKER_LOCKED_IN;
	posR =  UPPERLOCKER_RELEASED_IN;
	tx_buf[0] = positioncheck(posL, posR);
	
	posL = 	LOWERLOCKER_LOCKED_IN;
	posR =  LOWERLOCKER_RELEASED_IN;
	tx_buf[1] = positioncheck(posL, posR);
	
	posL = 	UPPERDOOR_OPEN_IN;
	posR =  UPPERDOOR_CLOSE_IN;
	tx_buf[2] = positioncheck(posL, posR);	

	posL = 	LOWERDOOR_OPEN_IN;
	posR =  LOWERDOOR_CLOSE_IN;
	tx_buf[3] = positioncheck(posL, posR);
	
	posL = 	LIFTER_TOP_IN;
	posR =  LIFTER_BOTTOM_IN;
	tx_buf[4] = positioncheck(posL, posR);	
	
	AppResultRes(CMD_POS_MAINPART,0 , tx_buf, 5);
}



void AppLed(u8 * lcmd)
{
	u8 temp;
	if(gGlobal.m_AppDataLen != 2)
	{
			SendErrCode(ERR_FORMAT, CMD_LED_CTRL);
			return;
	}
	temp = *lcmd;
	if(temp != 1 || temp != 2 || temp != 3)
	{
		SendErrCode(ERR_CMD_DATAERROR, CMD_LED_CTRL); 
		return;	
	}
	else
	{
		gGlobal.m_LED.ID = temp;
		temp = (*lcmd++);
		gGlobal.m_LED.status = temp;
	}
	AppOperationRes(CMD_LED_CTRL);
	xEventGroupSetBits( (EventGroupHandle_t) EventGroupHandle, (EventBits_t) LED_EVENTBIT);
}


/*
	function is used to operate the lockers.
[input] command:  command = 1 operate the upper locker and release it
									command = 2 operate the upper locker and lock it
									command = 3 operate the lower locker and release it
									command = 4 operate the lower locker and lock it

*/
void AppLocker( u8 command, u8 ctcmd)
{
   
	if(gGlobal.m_AppDataLen != 1)
	{
			SendErrCode(ERR_FORMAT, CMD_LOCKER_UPPERLOWER);
			return;
	}
	
	if (command == 1)
	{
		/* to release the upperlocker */
		gGlobal.m_LOCKERUpper.status = CLOCKWISE;
		
	}
	else if (command ==2)
		gGlobal.m_LOCKERUpper.status = COUNTERCLOCKWISE;
	else if (command == 3)
		gGlobal.m_LOCKERLower.status = CLOCKWISE;
	else if (command == 4)
		gGlobal.m_LOCKERLower.status = COUNTERCLOCKWISE;
	else
	{
		SendErrCode(ERR_CMD_DATAERROR, ctcmd); 
		return;
	}			
	AppOperationRes(ctcmd);
	gGlobal.m_CAN.locker_ready = 1;  /* locker task is in use*/
	xEventGroupSetBits( (EventGroupHandle_t) EventGroupHandle, (EventBits_t) LOCKER_EVENTBIT);

}

void AppLifter(u8 cmd, u8 ltcmd)
{
		if(gGlobal.m_AppDataLen != 1)
    {
        SendErrCode(ERR_FORMAT, CMD_LOCKER_DOOR);
        return;
    }
		if(cmd == 1)
		{
			gGlobal.m_stack.operationDoorAcc.stepper_destination = 1;
		}
		else if(cmd == 2)
		{
			gGlobal.m_stack.operationDoorAcc.stepper_destination = 2;
		}
		else if(cmd == 3)
		{
			gGlobal.m_stack.operationDoorAcc.stepper_destination = 3;
		}
		else
		{
			gGlobal.m_stack.operationDoorAcc.stepper_destination = 0;
			SendErrCode(ERR_CMD_DATAERROR, ltcmd); 
			return;
		}
		AppOperationRes(ltcmd);		
		gGlobal.m_CAN.lifter_ready = 1;
		xEventGroupSetBits( (EventGroupHandle_t) EventGroupHandle, (EventBits_t) LIFTER_EVENTBIT);

}

void AppDoor(u8 cmd1,u8 cmd2,u8 cmd3, u8 dtcmd)
{
		if(gGlobal.m_AppDataLen != 3)
    {
        SendErrCode(ERR_FORMAT, CMD_LOCKER_DOOR);
        return;
    }
		/* single door mode*/
		if (cmd1 == 1)  
		{
			//gGlobal.m_CAN.operation_mode = 1;
			//��������  example: 01 01 00 01(open upper door);01 01 00 02 (close the upper door)
			if(cmd2 == 1)  
			{
				gGlobal.m_stack.operationID = 1;
				//���Ż��߹���
				switch (cmd3)
				{
					case 1:
							gGlobal.m_stack.operationDIR = 0x11; //open the door = clockwise
						break;
					case 2:
							gGlobal.m_stack.operationDIR  = 0x12; //close the door
						break;
					default:
							SendErrCode(ERR_CMD_DATAERROR, dtcmd);
						break;
				}
			
			}	
			//�������� 
			//example: 01 00 01 01 (open the lower door); 01 00 01 02��close the lower door)
			else if(cmd2 == 2)
			{
				gGlobal.m_stack.operationID = 2;
				// ���Ż��߹���
				switch(cmd3)
				{
					case 1:
						gGlobal.m_stack.operationDIR = 0x11;
					break;
					
					case 2:
						gGlobal.m_stack.operationDIR = 0x12;
					break;		
					
					default:
						SendErrCode(ERR_CMD_DATAERROR, dtcmd);
						break;
				}
			}
			else
			{
				SendErrCode(ERR_CMD_DATAERROR, dtcmd); 
				return;
			}
		}		
		/* dual door mode*/
		else if(cmd1 == 2)
		{
			gGlobal.m_stack.operationID = 3;
			if (cmd3 == 1)
				gGlobal.m_stack.operationDIR = 0x11;  //door open
			else if(cmd3 == 2)
				gGlobal.m_stack.operationDIR = 0x12;  //door close
			else
			{
				SendErrCode(ERR_CMD_DATAERROR, dtcmd); 
				return;
			}
		}
		else
		{
			SendErrCode(ERR_CMD_DATAERROR, dtcmd); 
			return;
		}
		AppOperationRes(dtcmd);
		
		xEventGroupSetBits( (EventGroupHandle_t) EventGroupHandle, (EventBits_t) DOOR_EVENTBIT);
}

/*Ӧ�ô���ľ���ʵ��*/
void ProcessUpCmd(All_PACK *pack)
{
    BYTE TotalLengthByte;
    BYTE FlagByte;
    BYTE DataLenByte;
    BYTE btCmd, Data[4];
		u8 i;
		pack->m_Pos = 2;
    BYTEFromPack((*pack),TotalLengthByte); // fetch length of the data
    BYTEFromPack((*pack),FlagByte);   // fetch Flag from the data pack  
    if(FlagByte != CMD_BAG_FLAG_ANZHUO2WAISHE)
    {
        SendErrCode(ERR_OPERATION,cmd);
        return; 
    }
    BYTEFromPack((*pack),DataLenByte);
    BYTEFromPack((*pack),btCmd);
		for (i=0;i<DataLenByte; i++)
		{
			BYTEFromPack((*pack),Data[i]);
		}
		
		/*
		1. pack->m_Len is the total bytes being received from the USART
		2. TotalLengthByte is the third byte of the received data
		3. DataLenByte is the 5th byte of the received data 
		*/
    if( (TotalLengthByte > PACK_SIZE) || (pack->m_Len != (TotalLengthByte+4/* header+length�ֽ�+У���ֽ�*/) ) || (TotalLengthByte != (DataLenByte+3)) )//Length + Payload(Flag + Len + Cmd + Data)
    {                                                                                                                                            //  0x09 +  		0x00 + 0x06 + 0x15 + 'UPDATE'
        SendErrCode(ERR_OPERATION, cmd);
        return; 
    }                                                                              
    gGlobal.m_AppDataLen = DataLenByte;
		
    switch(btCmd)
	{
		case CMD_PING_TEST://ping֡��������
				AppPingTest(pack);
				break;
		case CMD_READ_DEVICEINFOR://��ȡ�豸�ͺ�����
				AppReadDeviceinfor(pack);
				break;
		case CMD_RESET_DEVICE://��λ����
				AppRestDevice(pack);
				break;
		case CMD_START_RESPOND://����������������
				AppStartRespond(pack);
				break;
		case CMD_END_RESPOND://ֹͣ������������
				AppEndRespond(pack);
				break;     
		case CMD_START_UPDATA://��ʼ��������
				AppStartUpdata(pack);
				break;	

		case CAM_DEBUG:
				DEBUG = Data[0];
		break;

		case CMD_LOCKER_UPPERLOWER:
				if (gGlobal.m_CAN.locker_ready == RESET && gGlobal.m_CAN.door_ready == RESET)
					AppLocker( Data[0], CMD_LOCKER_UPPERLOWER);
				else
					SendErrCode(ERR_CMD_ERROR_BUSY, cmd);  
		break;
				
		case CMD_LOCKER_DOOR:
			if(gGlobal.m_CAN.door_ready == RESET && gGlobal.m_CAN.locker_ready == RESET)
			{
				AppDoor(Data[0],Data[1],Data[2], cmd);
			}
			else
				SendErrCode(ERR_CMD_ERROR_BUSY, cmd);  
		break;
			
		case CMD_MOTOR_LIFTER:
			if(gGlobal.m_CAN.lifter_ready == RESET)
			{
				AppLifter(Data[0], cmd);
			}
			else
				SendErrCode(ERR_CMD_ERROR_BUSY, cmd);  
		break;
			
		case CMD_LED_CTRL:
			AppLed(Data);
		break;
		
		case CMD_POS_MAINPART:
			AppPosCheck();
		break;
		
		case CMD_ADC_CALIBRA:
			AppAdcCalibration();
		break;
		
		case CMD_HAND_PROT_THRESHOLD:
			AppHandProtection(Data);
		break;
			
		default:
				SendErrCode(ERR_CMD, cmd);//�������
				break;
	}
}


void UsartFun(void)
{
	u8 btComIndex, btPackIndex, chksum, xorlen;
	if(gGlobal.m_UsartData.p_flag==0x55)
	{
		btComIndex = 1;
		btPackIndex =1;	
		cmd = gGlobal.m_ComSet[btComIndex].m_RecvPack[btPackIndex].m_pBuffer[5]; // cmd ����
	//	memcpy(gGlobal.m_ComSet[btComIndex].m_RecvPack[btPackIndex].m_pBuffer,gGlobal.m_UsartData.p_buffer,gGlobal.m_UsartData.p_length);
	//	gGlobal.m_ComSet[btComIndex].m_RecvPack[btPackIndex].m_Len = gGlobal.m_UsartData.p_length;
		xorlen = gGlobal.m_ComSet[btComIndex].m_RecvPack[btPackIndex].m_Len - 3;
		chksum = CalcXORSum(&gGlobal.m_ComSet[btComIndex].m_RecvPack[btPackIndex].m_pBuffer[2],xorlen);
		if(chksum == gGlobal.m_ComSet[btComIndex].m_RecvPack[btPackIndex].m_pBuffer[xorlen+2])
		{
			ProcessUpCmd(&gGlobal.m_ComSet[btComIndex].m_RecvPack[btPackIndex]);
			gGlobal.m_UsartData.p_flag=0;
		}	
		else
		{
			SendErrCode(ERR_XOR,cmd);
			memset(gGlobal.m_UsartData.p_buffer,0,sizeof(gGlobal.m_UsartData.p_buffer));
			memset(gGlobal.m_ComSet[btComIndex].m_RecvPack[btPackIndex].m_pBuffer,0,181);
			gGlobal.m_UsartData.p_length = 0;
			
		}
	}
}

void USART1_IRQHandler(void)                	//����2�жϷ������
	{
		u8 Res;
		BaseType_t xHigherPriorityTaskWoken, Result;
		if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)  
		{
			__nop(); 
			__nop(); 
			__nop(); 
			Res=USART1->SR;
			Res=USART1->DR;
		  DMA_Cmd(DMA1_Channel5,DISABLE);  
			
      Res = PACK_SIZE - DMA_GetCurrDataCounter(DMA1_Channel5);
			gGlobal.m_UsartData.p_length = Res;
			gGlobal.m_UsartData.p_flag=0x55;
			memcpy(gGlobal.m_ComSet[1].m_RecvPack[1].m_pBuffer,gGlobal.m_UsartData.p_buffer,gGlobal.m_UsartData.p_length);
			gGlobal.m_ComSet[1].m_RecvPack[1].m_Len = gGlobal.m_UsartData.p_length;
      DMA_SetCurrDataCounter(DMA1_Channel5,PACK_SIZE);  
      DMA_Cmd(DMA1_Channel5,ENABLE);  
			Result = xSemaphoreGiveFromISR(BinarySemaphore_USART, &xHigherPriorityTaskWoken);
			if(Result == pdTRUE)
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
} 
	


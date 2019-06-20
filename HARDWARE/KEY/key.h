#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
#include "header.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//ALIENTEKս��STM32������
//������������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/3
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) �������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////   	 


//#define KEY0 PEin(4)   	//PE4
//#define KEY1 PEin(3)	//PE3 
//#define KEY2 PEin(2)	//PE2
//#define WK_UP PAin(0)	//PA0  WK_UP

#define KEY0  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15)//��ȡ����0
//#define KEY1  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)//��ȡ����1
//#define KEY2  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2)//��ȡ����2 
//#define WK_UP   GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)//��ȡ����3(WK_UP) 

 

#define KEY0_ShutDown 	1	//KEY0����
#define KEY0_PowerUp    2 
//#define KEY1_PRES	2	//KEY1����
//#define KEY2_PRES	3	//KEY2����
//#define WKUP_PRES   4	//KEY_UP����(��WK_UP/KEY_UP)

void ENSWITCH_Init(void);
void KEY_Init(void);//IO��ʼ��
void RESET_Motor(void);
void RESET_24VOUT(void);
void RESET_Power(void);
void Detection_Motor(void);
void Detection_24VOUT(void);
void Detection_Power(void);
u8 KEY_Scan(u8);  	//����ɨ�躯��					    
#endif
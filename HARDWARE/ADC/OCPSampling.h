#ifndef __OCPSAMPLING
#define __OCPSAMPLING

#include "header.h"

#define LifterChannel_IO_IN()   {GPIOC->CRH&=0xFFFFFF0F;GPIOC->CRH|=4<<4;}
#define LifterChannel_IO_OUT()  {GPIOC->CRH&=0xFFFFFF0F;GPIOC->CRH|=3<<4;}
#define Lifter_OUT_OFF()						{PCout(9) = 0;}

#define LockerChannel_IO_IN()   {GPIOB->CRH&=0xF0FFFFFF;GPIOB->CRH|=4<<24;}
#define LockerChannel_IO_OUT()  {GPIOB->CRH&=0xF0FFFFFF;GPIOB->CRH|=3<<24;}
#define Locker_OUT_OFF()						{PBout(14)=0;}

#define UpperMotorChannel_IO_IN()   {GPIOB->CRL&=0xFFFFFFF0;GPIOB->CRL|=4;}
#define UpperMotorChannel_IO_OUT()  {GPIOB->CRL&=0xFFFFFFF0;GPIOB->CRL|=3;}
#define UpperMotor_OUT_OFF()						{PBout(0)=0;}

#define LowerMotorChannel_IO_IN()   {GPIOA->CRL&=0xFFF0FFFF;GPIOA->CRL|=4<<16;}
#define LowerMotorChannel_IO_OUT()  {GPIOA->CRL&=0xFFF0FFFF;GPIOA->CRL|=3<<16;}
#define LowerMotor_OUT_OFF()						{PAout(4)=0;}




void VoltSampling_Init(void);
void OCPChannel_Init(void);
void ONPOSITIONChannel_Init(void);
void FetchingCurrentValue(void);
void LockerChannelEnable(void);
void LockerChannelDisable(void);
#endif

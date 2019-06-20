/*=====================================================================================
 File name:        PID_REG3.C  (IQ version)                  
                    
 Originator:	Digital Control Systems Group
			Texas Instruments

 Description:  The PID controller with anti-windup                   

=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2005	Version 3.20
-------------------------------------------------------------------------------------*/

#include "pid_reg3.h"
#include "IQmathLib.h"
void pid_reg3_calc(PIDREG3 *v)
{	
    // Compute the error
    v->Err = v->Ref - v->Fdb;

    // Compute the proportional output
    v->Up = _IQmpy(v->Kp,v->Err);

    // Compute the integral output
    v->Ui = v->Ui + _IQmpy(v->Ki,v->Up) + _IQmpy(v->Kc,v->SatErr);

    // Compute the derivative output
    v->Ud = _IQmpy(v->Kd,(v->Up - v->Up1));

    // Compute the pre-saturated output
    v->OutPreSat = v->Up + v->Ui + v->Ud;
//	v->OutPreSat =v->OutPreSat +_IQ(0.06);


    // Saturate the output
    if (v->OutPreSat > v->OutMax)
      v->Out =  v->OutMax;
    else if (v->OutPreSat < v->OutMin)
      v->Out =  v->OutMin;
    else
      v->Out = v->OutPreSat;

    // Compute the saturate difference
    v->SatErr = v->Out - v->OutPreSat;

    // Update the previous proportional output
    v->Up1 = v->Up;  
}

//void PID_SPEED_Coefficients_Update(PID_Struct_t *p,PIDREG3 *v)
//{
//	_iq hTMP;
//	if(v->Fdb<_IQ(Speed_min))
//	{
//		p->hKp_Gain=_IQ(Kp_Fmin);
//		p->hKi_Gain=_IQ(Ki_Fmin);
//		#ifdef DIFFERENTIAL_TERM_ENABLED
//		p->hKd_Gain=_IQ(Kd_Fmin);
//		#endif
//	}
//	else if (v->Fdb<_IQ(Speed_F1))
//	{
//		hTMP=v->Fdb-_IQ(Speed_min);
//		p->hKp_Gain=_IQmpy(_IQ(alpha_KP),hTMP);
//		p->hKp_Gain+=_IQ(Kp_Fmin);
//		p->hKi_Gain=_IQmpy(_IQ(alpha_KI),hTMP);
//		p->hKi_Gain+=_IQ(Ki_Fmin);
//		#ifdef DIFFERENTIAL_TERM_ENABLED
//		p->hKd_Gain=_IQmpy(_IQ(alpha_KD),hTMP);
//		p->hKd_Gain+=_IQ(Kd_Fmin);
//		#endif	
//	}
//	else
//	{
//		p->hKp_Gain=_IQ(Kp_F1);
//		p->hKi_Gain=_IQ(Ki_F1);
//		#ifdef DIFFERENTIAL_TERM_ENABLED
//		p->hKd_Gain=_IQ(Kd_F1);
//		#endif		
//	}
//	v->Kp=p->hKp_Gain;
//	v->Kd=p->hKd_Gain;
//	v->Ki=p->hKi_Gain;
//	v->Kc=Kc_Fmin;


//}

//void PID_SPEED_calc(PID_Struct_t *p,PIDREG3 *v)
//{
//	_iq wError,wProportional_term,wIntegral_term,wAux;
//	#ifdef DIFFERENTIAL_TERM_ENABLED
//	_iq wDifferential_term;
//	#endif

//	wError=v->Ref-v->Fdb;
//	wProportional_term=_IQmpy(p->hKp_Gain,wError);
//	if(p->hKi_Gain==0)
//		p->wIntegral=0;
//	else
//	{
//		wIntegral_term=_IQmpy(p->hKi_Gain,wError);
//		wAux=p->wIntegral+wIntegral_term;
//		if(wAux>p->wUpper_Limit_Integral)
//			p->wIntegral=p->wUpper_Limit_Integral;
//		else if(wAux<p->wLower_Limit_Integral)
//			p->wIntegral=p->wLower_Limit_Integral;
//		else
//			p->wIntegral=wAux;
//	}
//	#ifdef DIFFERENTIAL_TERM_ENABLED
//	{
//		_iq wtemp;
//		wtemp=wError-p->wPreviousError;
//		wDifferential_term=_IQmpy(p->hKd_Gain,wtemp);
//		p->wPreviousError=wError;
//		v->Out=_IQdiv(wProportional_term,p->hKp_Divisor);
//		v->Out+=_IQdiv(p->wIntegral,p->hKi_Divisor);
//		v->Out+=_IQdiv(wDifferential_term,p->hKd_Divisor);
//	}
//	#else
//	{
//		v->Out=_IQdiv(wProportional_term,p->hKp_Divisor);
//		v->Out+=_IQdiv(p->wIntegral,p->hKi_Divisor);
//	}
//	#endif

//	if(v->Out>p->hUpper_Limit_Output)
//		v->Out=p->hUpper_Limit_Output;
//	else if(v->Out<p->hLower_Limit_Output)
//		v->Out=p->hLower_Limit_Output;
//	else
//		return;
//}

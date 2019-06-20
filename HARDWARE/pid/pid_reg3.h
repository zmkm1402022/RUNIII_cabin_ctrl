/* =================================================================================
File name:       PID_REG3.H  (IQ version)                    
                    
Originator:	Digital Control Systems Group
			Texas Instruments

Description: 
Header file containing constants, data type definitions, and 
function prototypes for the PIDREG3.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2005	Version 3.20
------------------------------------------------------------------------------*/
#include "IQmathLib.h" 
#include "stdint.h"
#ifndef __PIDREG3_H__
#define __PIDREG3_H__

typedef struct {  _iq  Ref;   			// Input: Reference input 
				  _iq  Fdb;   			// Input: Feedback input 
				  _iq  Err;				// Variable: Error
				  _iq  Kp;				// Parameter: Proportional gain
				  _iq  Up;				// Variable: Proportional output 
				  _iq  Ui;				// Variable: Integral output 
				  _iq  Ud;				// Variable: Derivative output 	
				  _iq  OutPreSat; 		// Variable: Pre-saturated output
				  _iq  OutMax;		    // Parameter: Maximum output 
				  _iq  OutMin;	    	// Parameter: Minimum output
				  _iq  Out;   			// Output: PID output 
				  _iq  SatErr;			// Variable: Saturated difference
				  _iq  Ki;			    // Parameter: Integral gain
				  _iq  Kc;				// Parameter: Integral correction gain
				  _iq  Kd; 		        // Parameter: Derivative gain
				  _iq  Up1;		   	    // History: Previous proportional output
		 	 	  void  (*calc)();	  	// Pointer to calculation function
				 } PIDREG3;	            

extern PIDREG3 pid1_idc;
				 
typedef struct {  
				  _iq hKp_Gain;
				  _iq hKp_Divisor;
				  _iq hKi_Gain;
				  _iq hKi_Divisor;  
				  _iq hLower_Limit_Output;     //Lower Limit for Output limitation
				  _iq hUpper_Limit_Output;     //Lower Limit for Output limitation
				  _iq wLower_Limit_Integral;   //Lower Limit for Integral term limitation
				  _iq wUpper_Limit_Integral;   //Lower Limit for Integral term limitation
				  _iq wIntegral;
				  // Actually used only if DIFFERENTIAL_TERM_ENABLED is enabled in
				  //stm32f10x_MCconf.h
				  _iq hKd_Gain;
				  _iq hKd_Divisor;
				  _iq wPreviousError;
				  void (*calc1)();
				  void (*calc2)();
				} PID_Struct_t;

typedef PIDREG3 *PIDREG3_handle;
typedef PID_Struct_t *PID_Struct_t_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the PIDREG3 object.
-----------------------------------------------------------------------------*/        
#define Speed_min (float)0.3
#define Speed_F1  (float)0.83
#define Kc_Fmin	  (float)0.0002
#define Kp_Fmin   (float)0.1
#define Ki_Fmin	  (float)0.0001
#define Kd_Fmin   (float)0.0001
#define Kp_F1     (float)0.8
#define Ki_F1     (float)0.0007
#define Kd_F1     (float)0.0007
#define alpha_KP  (float)((Kp_F1-Kp_Fmin)/(Speed_F1-Speed_min))
#define alpha_KI  (float)((Ki_F1-Ki_Fmin)/(Speed_F1-Speed_min))
#define alpha_KD  (float)((Kd_F1-Kd_Fmin)/(Speed_F1-Speed_min))
				
#define PIDREG3_DEFAULTS { 0, \
                           0, \
                           0, \
                           _IQ(1.3), \
                           0, \
                           0, \
                           0, \
                           0, \
                           _IQ(0.6), \
                           _IQ(0.04), \
                           0, \
                           0, \
                           _IQ(0.02), \
                           _IQ(0.5), \
                           _IQ(1.05), \
                           0, \
              			  (void (uint32_t *))pid_reg3_calc }
#define PID_Struct_t_DEFAULTS { _IQ(0.001),\
								_IQ(10),\
								_IQ(0.001),\
								_IQ(10),\
								_IQ(0.6),\
								_IQ(0.05),\
								_IQ(0.2),\
								_IQ(0.01),\
								_IQ(0),\
								_IQ(0.001),\
								_IQ(10),\
								_IQ(0),\
								(void (*)(uint32_t)) PID_SPEED_calc,\
								(void (*)(uint32_t)) PID_SPEED_Coefficients_Update}



/*------------------------------------------------------------------------------
Prototypes for the functions in PIDREG3.C
------------------------------------------------------------------------------*/
void pid_reg3_calc(PIDREG3_handle);
void PID_SPEED_calc(PID_Struct_t_handle,PIDREG3_handle);
void PID_SPEED_Coefficients_Update(PID_Struct_t_handle,PIDREG3_handle);
#endif // __PIDREG3_H__

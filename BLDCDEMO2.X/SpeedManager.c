/************************************************************************
*                                                                       *
*     Project              : 3-Phase Brushless Motor Control            *
*                                                                       *
*     Author               : w.r.brown                                  *
*                                                                       *
*     Company              : Microchip Technology Incorporated          *
*     Filename             : SpeedManager.c                             *
*     Date                 : 2009/03/11                                 *
*     SharePoint Version   : 1.4                                        *
*                                                                       *
*     Other Files Required :                                            *
*     Tools Used: MPLAB GL : 8.14                                       *
*                 Compiler : Hi-Tech                                    *
*                 Assembler:                                            *
*                 Linker   :                                            *
*                                                                       *
*************************************************************************
*
////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                    //
//   Request for motor speed is passed in as a 10-bit unsigned integer.                               //
//   This routine converts the request to a number from 0 to 255.                                     //
//   That number is passed to the GetCCPVal() routine which looks up the appropriate duty cycle       //
//   in the SpeedProfile table and sets CCPR1L to the corresponding duty cycle.                       //
//                                                                                                    //
////////////////////////////////////////////////////////////////////////////////////////////////////////
*
*
*******************************************************************************************************
* Software License Agreement:
*
* The software supplied herewith by Microchip Technology Incorporated
* (the "Company") for its PICmicro® Microcontroller is intended and
* supplied to you, the Company's customer, for use solely and
* exclusively on Microchip PICmicro Microcontroller products. The
* software is owned by the Company and/or its supplier, and is
* protected under applicable copyright laws. All rights are reserved.
* Any use in violation of the foregoing restrictions may subject the
* user to criminal sanctions under applicable laws, as well as to
* civil liability for the breach of the terms and conditions of this
* license.
*
* THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
* WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
* TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
* PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
* IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
* CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
*******************************************************************************************************
*
*
*******************************************************************************************************
*
* Change History:
* Author               Date        Comment
* w.r.brown            2009.03.26  V 1.4  - moved GetCCPVal() to ...main()
* w.r.brown            2009.03.24  V 1.3  - expanded GetCCPVal() for full 10-bit resolution
* w.r.brown            2009.03.15  V 1.2  - added running average
* w.r.brown            2009.03.12  V 1.1  - made ramped_speed and external global to be set during initialization
* w.r.brown            2009.03.11  V 1.0  - changed version to SharePoint version number
* w.r.brown            2009.03.11  V 0.1  - added lookup/set CCPR1 via speed profile table
*                                           added header information
* w.r.brown            2009.03.07  V 0.0  - Split off of BLCD_Main...c
*******************************************************************************************************/
#include <xc.h>
#include "BLDC.h"

extern bit supply_is_valid;
extern bit stop_flag;
extern bit startup_complete_flag;
extern bit run_flag;
extern bit TMR0_duty_flag;
extern char TMR0_duty_timer;
extern unsigned char ramped_speed;

/************************************************************************
*                                                                       *
*      Function: 	     SpeedManager                                   *
*                                                                       *
*      Description:    Determine desired speed from                     *
*                      voltage on ADC channel                           *
*                                                                       *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/
//    PWM duty cycle (effective voltage to motor) is set by reading
//    ADC (presumably a speed control pot on a pin) and the PWM duty cycle
//    (ie. motor voltage) is ramped up/down to the ADC reading by 
//    incrementing/decrementing the duty cycle register once every N ms
//    (determined by the constant TIMEBASE_DUTY_RAMP).
//    The motor voltage is preset to a low voltage (% of max determined by
//    STARTUP_DUTY_CYCLE) at startup and prevented from changing during the 
//    startup period.

void SpeedManager(int speed)
{
   unsigned char speedrequest;
   static unsigned int srsum = 0;
   static unsigned int sravg = 0;
   
	//if(!supply_is_valid) return;           // exit if motor supply out of range
	if(!TMR0_duty_flag) return;            // hold off for 10 ms

	TMR0_duty_flag=0;
	if(--TMR0_duty_timer) return;          // wait until timer times out

	TMR0_duty_timer = TIMEBASE_DUTY_RAMP;  // reset timer
    
    if(speed > 1023) speed = 1023;
    
  	speedrequest = speed >> 2; // provide an 8-bit speed control
	// stop and prevent run when speed control is below the minimum speed threshold
   
   // running average
   srsum -= sravg;
   srsum += speedrequest;
   // sum divisor determines size of average
   sravg = srsum>>ADC_AVG_FACTOR;
   
   // stop-run hysterisis: stop when below low threshold   
	if(sravg < REQUEST_OFF)
	{
   	if(run_flag) stop_flag = 1;        // this forces a system reset stop during run
   	run_flag = 0;
   	return;
   }
   // stop-run hysterisis: don't start until above high threshold
   if(sravg > REQUEST_ON) run_flag = 1;
   
	if(!startup_complete_flag) return;     // do not start to accelerate until startup is complete
   
	// ramp up or down to the requested speed setting
	// NOTE: if the speedrequest-average sample size is sufficently large then this
	//       ramping function can be eliminated.
	if(sravg > ramped_speed) ramped_speed++;
	if(sravg < ramped_speed) ramped_speed--;
	
   // set the motor voltage PWM by accessing values in a table
   // indexed by the global variable ramped_speed
	GetCCPVal(ramped_speed);

	return;
}

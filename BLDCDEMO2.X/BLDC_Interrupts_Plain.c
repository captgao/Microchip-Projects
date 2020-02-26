/************************************************************************
*                                                                       *
*     Project              : 3-Phase Brushless Motor Control            *
*                                                                       *
*     Author               : W.R.Brown                                  *
*                                                                       *
*     Company              : Microchip Technology Incorporated          *
*     Filename             : BLDC_Interrupts_Plain.c                    *
*     Date                 : 2009/03/12                                 *
*     SharePoint Version   : 1.1                                        *
*                                                                       *
*     Tools Used: MPLAB GL : 8.14                                       *
*                 Compiler : Hi-Tech                                    *
*                 Assembler:                                            *
*                 Linker   :                                            *
*                                                                       *
*************************************************************************
*
////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                    //
//   Plain vanilla interrupts with two states: Zero_detect and Commutate.                             //
//   This version also includes dynamic blanking.                                                     //
//                                                                                                    //
////////////////////////////////////////////////////////////////////////////////////////////////////////
*
*
*********************************************************************
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
*********************************************************************
*
* Change History:
* Author               Date        Comment
* w.r.brown            2011.03.07  V 1.2  - Moved TMR1IF-clear to end of ISR
* w.r.brown            2009.03.22  V 1.1  - Added voltage sensing to stall detection
* w.r.brown            2009.03.15  V 1.0  - First release
*******************************************************************************************************/
#include <xc.h>
#include "BLDC.h"
#include "EBM_Motor.h"
#include "1937_DRIVER.h"

/************************************************************************
* variable definitions                                                  *
*************************************************************************/

extern doublebyte TMR1_comm_time;
extern char TMR0_stall_timer;

extern bit startup_complete_flag;
extern bit stop_flag;
extern bit init_complete_flag;
extern bit rising_bemf_flag;
extern bit measure_bemf_flag;

enum {high_res_setup,zero_detect,commutate}isr_state;

extern int zc_error;
extern int temp;
extern doublebyte expected_zc;
extern doublebyte zc;
extern doublebyte comm_after_zc;
extern char tach_timer;
extern char comm_state;

int CommOffset;

/************************************************************************
*                                                                       *
*                          I N T E R R U P T                            *
*                                                                       *
*************************************************************************/
////////////////////////////////////////////////////////////////////////////////////////////////////////
//    Motor drive is controlled by interrupts. There are two types of interrupts:                     //
//    Timer1 overflow and BEMF comparator.                                                            //
//                                                                                                    //
//    Timer1 interrupts:                                                                              //
//       Commutation interrupt - Motor drive is changed to the next commutation phase.                //
//       This is also the start of the blanking time when the upcomming period is rising BEMF (for    //
//       high-side modulation) or falling BEMF (for low-side modulation).                             //
//       Timer1 is preloaded to overflow after the commutation time. If the BEMF_FLAG is set then     //
//       blanking is performed and then the the comparator interrupt is enabled after clearing the    //
//       comparator interrupt-on-change registers otherwise commutation correction calculations       //
//       are performed basd on the previous zero-crossing event data.                                 //
//    Comparator interrupt:                                                                           //
//       Zero Cross interrupt - This signals when the BEMF voltage has crossed the motor supply       //
//       voltage midpoint. Timer1 is read and the measured time is compared to the expected time.     //
//       The error time difference is used to adjust both the upcomming commutation and the predicted //
//       commutation period for future commutations. Commutation computations are performed in the    //
//       next commutation state when there is more time because blanking and zero cross are not       //
//       performed.                                                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////////
void __interrupt() ISR(void) {
   char ctemp;
         
   switch (isr_state)
   {
      case zero_detect:
         // disable comparator interrupts
         CxIE = 0;
         // this is either a Timer1 interrupt or a comparator interrupt
         // if it's a comparator interrupt then zero cross has been detected and needs to be processed
         // if it's a Timer1 interrupt then the zero cross was missed so we need to skip zero cross
         // processing and fall through to commutation.
         if(CxIF)
         {
            TP2=1;                          //    Diagnostic - Zero detection
            do{
               zc.bytes.high = TMR1H;
               zc.bytes.low = TMR1L;
            }
            while (zc.bytes.high != TMR1H);
            
            if(startup_complete_flag)
            {
               TMR1ON = 0;
               TMR1H = comm_after_zc.bytes.high;
               TMR1L = comm_after_zc.bytes.low;
               TMR1ON = 1;
            }
            TP2 = 0;  // Diagnostic
            TMR1IF = 0;
            isr_state = commutate;
            break;
         }
         // if execution reaches this point then no zero cross was detected
         // this only happens during deceleration and when searching for zero cross in forced commutation
         // if we're decelerating then adjust the commutation time to catch up with the motor
         
          if(startup_complete_flag)
          {
             // TMR1_comm_time is negative so adding negative number lengthens comm time.
             // shorten by 1/8 commutation cycle
             TMR1_comm_time.word += (expected_zc.word>>2);
          }
         
      case commutate:
         // Commutation occurs when Timer1 overflows
         if(TMR1IF)
         {
            //    This is the first Timer1 interrupt after zero cross detection or previous commutation event.
            //    At this point Timer1 has overflowed so it is time to commutate.
            //    This service commutates the motor and either waits for a blanking period or computes the next commutation.
            //    If BEMF_FLAG indicates that zero cross detection is to be performed in the the upcomming period then blanking is performed.
            //    Blanking holds off the input to the comparator to allow the commutation switching transients to settle.
            //    A relatively clean BEMF signal will then be available for zero cross detection.
            //    In this version of the software blanking is performed dynamically. We wait a minimum blanking period to
            //    give the drivers a chance to settle then we read the comparator output until it is low thereby ensuring
            //    that the flyback currents have settled. Then, the comparator will be setup for zero cross detection.
            Commutate();
            if(BEMF_FLAG)
            {
               // dynamic blanking
               // wait a mimimum blanking time to allow drivers to settle
               while(TMR1L < BLANKING_COUNT_us);
               // setup Timer1 for the full commutation period
               TMR1ON = 0;
               TMR1H = TMR1_comm_time.bytes.high;
               TMR1L += TMR1_comm_time.bytes.low;
               if(CARRY)
               {
                  TMR1H++;
                  TMR1L = TMR1L;
               }      
               TMR1ON = 1;
               // wait for flyback currents to settle before setting up comparator for interrupts
               // if Timer1 overflows while waiting then flyback voltage and zero cross were both
               //   missed in which case we need to commutate and try again.
               if (startup_complete_flag)
                  while(CxOUT) if (TMR1IF) break;  
               ctemp = CMxCON0;   // reading control register clears mis-match flops
               CxIF = 0;
               CxIE = 1;
               isr_state = zero_detect;
               //TP1 = 0; // Diagnostic
            }   
            else
            {
               // The period we are presently in does not sense BEMF so no time is consumed in blanking and zero cross detection.
               // This is the longest period without interruption so we will optimize performance by using this time
               // to compute the commutation interval from the zero cross data collected in the previous period.
               // Compute new commutation time from previous zero cross data:
               // expected_zc is the positive expected time to zero cross and positive remaining commutation time
               // actual_zc is the negative of the actual time remaining until the planned commutation
               // the difference between the expected and actual is found by adding actual_zc to expected_zc
               // that difference is the zero cross error which will be stored in zc_error
               // if zero cross is late then the remaining time till planned commutation will be shorter than expected
               // the result of adding the relatively smaller negative number to the expected number will be positive error
               // a late zero cross is the result of comm times being too short so time must be added
               // time is added to the commutation time by making the Timer1 preset (TMR1_comm_time) smaller
               // therefore the error should be subtracted from TMR1_comm_time to make the correction
               // |                                                                       |
               // |<-------------------------(TMR1_comm_time)---------------------------->|
               // |<---------expected_zc------------->|<-------(-expected_zc)-------------|
               // |                             ----->|  zc_error  |<----(-actual_zc)-----|
               // |=(-2X)                             |=(-X)       |=(-N)
               // |                                                |<---- time to next comm = X ----->|
               // |<------------------- next TMR1_comm_time += zc_error/4 ------------------>|
               // -2X is Timer1 preset for comm time
               // -X is the expected Timer1 value at zero cross
               // -N is actual Timer1 value at zero cross
               
               // expected_zc has an MSb OR'd in because the result of the shift must always be negative
               // for example: times are computed by putting the negative of the count in the timer. When
               // the timer overflows the time is up. For counts 0x8000 and below the negative is 0x8nnn and
               // half that is 0x8nnn shifted right with sign extension or 0xCnnn. For counts above 0x8000 the
               // sign bit is lost in the negation so a right shift will have the wrong sign extended. 
               expected_zc.word = (TMR1_comm_time.word>>1) | 0x8000; // expected_zc is negative expected time remaining
               
               zc.word += CommOffset;
               
               zc_error = zc.word - expected_zc.word;           // negative actual - negative expected  = positive too late
               temp = zc_error;
               if(temp & 0x8000) temp = ~temp+1;  // absolute value
               // stop forced commutation if zero cross detected within middle half of comm period
               if (temp < (-expected_zc.word>>1))
               {
                 startup_complete_flag = 1;
                 TMR0_stall_timer = TIMEBASE_STALL_COUNT;
                 TP0 = 0; // Diagnostic
               }
               else
               {
                 TP0 = 1;  // Diagnostic
               }
               
               // Systems ramp up at the maximum rate determined by the difference
               // between the zero cross event (which usually happens immediately after blanking)
               // and the middle of the expected commutation time.

               TMR1_comm_time.word -= (zc_error>>ERROR_SCALE);       // accumulate error
               comm_after_zc.word = TMR1_comm_time.word + FIXED_ADVANCE_COUNT;
               
               // setup for commutation
               TMR1ON = 0;
               TMR1H = comm_after_zc.bytes.high;
               TMR1L += comm_after_zc.bytes.low;
               if(CARRY)
               {
                  TMR1H++;
                  TMR1L = TMR1L;
               }      
               TMR1ON = 1;
               //if((unsigned int)TMR1_comm_time.word > MAX_TMR1_PRESET) stop_flag=1;

               isr_state = commutate;
               // setup for commutation time after zero cross event
               // half the commutation time is adjusted for motor advance timing
               // expected_zc is negative time to commutation event
               // ADVANCE_COUNT is positive time to advance
               // Adding positive number to negative time shortens the negative time
               comm_after_zc.word = expected_zc.word - CommOffset;
            }
            TMR1IF = 0;
         }
         break;
      default:
         stop_flag = 1;
         break;
   }/* end switch */

}/* end ISR */


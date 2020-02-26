/************************************************************************
*                                                                       *
*     Project              : 3-Phase Brushless Motor Control            *
*                                                                       *
*     Author               : W.R.Brown                                  *
*                        (original program structure: Loris Bianchin)   *
*     Company              : Microchip Technology Incorporated          *
*     Filename             : F1937_Main.c                               *
*     Date                 : 2009/03/12                                 *
*     SharePoint Version   : 2.0                                        *
*                                                                       *
*     Other Files Required : BLDC.h                                     *
*     Tools Used: MPLAB GL : 8.14                                       *
*                 Compiler : Hi-Tech                                    *
*                 Assembler:                                            *
*                 Linker   :                                            *
*                                                                       *
*************************************************************************

////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                    //
//                           Sensorless 3-phase brushless motor control                               //
//                                                                                                    //
// Hardware -                                                                                         //
// This program uses the steerable ECCP PWM to modulate the low sides                                 //
// of a 3-phase motor drive bridge. Resistor dividers on the motor phases                             //
// are used to scale the back EMF (BEMF) and drive voltages to levels within range                    //
// of the device comparator. The positive comparator input monitors the motor supply                  //
// voltage, and the negative comparator input monitors the BEMF. The comparator                       //
// input mux is used to steer the undriven motor phase to the negative comparator                     //
// input. Zero cross is detected when the comparator minus input crosses half the                     //
// motor supply voltage. Zero cross is detected during either falling or rising BEMF only.            //
// Rising BEMF is sensed in systems with high-side modulation. Falling BEMF is sensed in              //
// systems with low-side modulation.                                                                  //
// Motor drive PWM duty cycle is selected by reading the ADC level on an analog                       //
// input pin. Three timers are used. Timer0 controls the warmup time, startup time,                   //
// and times between maintenance events such as reading the ADC for speed control.                    //
// Timer2 sets the PWM period. Timer1 has two functions: Zero cross blanking after                    //
// commutation and timing from blanking or zero cross to commutation. Timer1 always                   //
// runs at a 0.5 uS period.                                                                           //
//                                                                                                    //
// Software -                                                                                         //
// There are 3 stages of operation which are controlled by the TimeBaseManager():                     //
// Warmup - Commences immediately after reset. Lasts for 400 mS. Device peripherals                   //
//    are initialized.                                                                                //
// Startup - Commences immediately after warmup. Lasts for 300 mS. I/O drivers                        //
//    are initialized, interrupts are enabled, and forced commutation begins.                         //
//    The motor drive PWM is fixed at the lowest duty cycle. The commutation rate                     //
//    starts slowly and ramps up based on the zero cross detection.                                   //
// Run - Commences immediately after startup. Commutation period is measured by reading               //
//    the Timer1 count at the zero cross event. All motor operation is interrupt driven.              //
//    There are two types of vectored interrupts:                                                     //
//    1. Timer1 commutation interrupt - Motor drive is changed to the next commutation phase.         //
//       The comm_time counter is cleared. If the new commutation period is a BEMF sense period       //
//       then this is also the start of the blanking time.                                            //
//       Timer1 is preloaded to interrupt after the blanking period of BLANKING_COUNT_us which        //
//       is a function of the motor characteristics.                                                  //
//       During the blanking period the PWM interrupts continue to occur but no BEMF measurements     //
//       are made.                                                                                    //
//    2. Timer1 Blanking interrupt - This signals the end of blanking. Blanking was                   //
//       started at the commutation event. Timer1 is preloaded to overflow at the predicted           //
//       commutation time. The comparator interrupt on change registers are cleared and the           //
//       comparator interrupt is enabled. The comparator reference monitors half the motor drive      //
//       voltage. The other comparator input is connected to the undriven motor phase.                //
//    3. Comparator Zero Cross interrupt - This signals when the BEMF voltage has dropped below       //
//       half the motor supply voltage. Timer1 is read and the measured time is compared to the       //
//       expected time. The error time difference is used to adjust both the up comming commutation   //
//       and the predicted commutation period for future commutations.                                //
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

* w.r.brown            2009.04.21  V 0.1  - First release
*******************************************************************************************************/
#include <xc.h>
#include "BLDC.h"
#include "1937_DRIVER.h"
#include "EBM_Motor.h"


void SpeedManager(void);

#define __MPLAB_ICD__    2

/************************************************************************
* variable definitions                                                  *
*************************************************************************/

unsigned char comm_state;

doublebyte TMR1_comm_time;

char slow_start_events;
unsigned char TMR0_slow_start_timer;
unsigned char TMR0_startup_timer;
unsigned char TMR0_stall_timer;
unsigned char TMR0_warmup_timer;
unsigned char TMR0_duty_timer;
unsigned char timebase_10ms;

bit TMR0_startup_flag;
bit TMR0_warmup_flag;
bit TMR0_slow_start_flag;
bit TMR0_stall_flag;
bit TMR0_duty_flag;
bit warmup_complete_flag;
bit startup_complete_flag;
bit slow_start_complete_flag;
bit stop_flag;
bit run_flag;
bit init_complete_flag;
bit rising_bemf_flag;
bit startup_in_progress;

int zc_error;
int temp;
doublebyte expected_zc;
doublebyte zc;
doublebyte comm_after_zc;
unsigned char ramped_speed;

char startup_dutycycle = MED_STARTUP_DUTYCYCLE;
unsigned int startup_rpm = (0xFFFF - COMM_TIME_INIT + 1);

extern enum {high_res_setup,zero_detect,commutate}isr_state;;
extern const int CCP_Values[256];

/************************************************************************
*                                                                       *
*                              M A I N                                  *
*                                                                       *
*************************************************************************/
void main(void)
{
   stop_flag = 1;
   while(1) {
      CLRWDT();
      if(stop_flag) InitSystem();
      TimeBaseManager();
      WarmUpControl();
      ControlSlowStart();
      ControlStartUp();
      StallControl();
      //SpeedManager();
      


   }   

} // end main

/************************************************************************
*                         E N D   M A I N                               *
*************************************************************************/

/************************************************************************
*                                                                       *
*      Function:       InitSystem                                       *
*                                                                       *
*      Description:    initialize system registers and variables        *
*                                                                       *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/

void InitSystem(void) {

   // disable interrupts
   PEIE=0;   
   GIE=0;

   // Setup internal oscillator frequency
   OSCCON = OSCCON_INIT;
   
   //OPTION=OPTION_INIT;   

    AIN1_SEL = 1;
    AIN3_SEL = 1;
    AIN4_SEL = 1;
    AIN5_SEL = 1;
    AIN7_SEL = 1;
    TRISA = 0xFF;
    TRISB = 0x8;
    TRISC = 0;
/*
#ifdef PORTD_INIT
   PORTD=PORTD_INIT;
   TRISD=TRISD_INIT;
#endif
   
#ifdef PORTE_INIT
   PORTE=PORTE_INIT;
   TRISE=TRISE_INIT;
#endif
*/
   // initialize ADC
   
   CCP1CON = 0;            //disable PWM

   // TIMER0 and related startup variables
   TMR0 = 0;
   T0IF = 0;
   OPTION_REG = OPTION_REG_INIT;
   TMR0_warmup_timer = TIMEBASE_WARMUP_COUNT;
   TMR0_slow_start_timer = TIMEBASE_SLOW_STEP;
   TMR0_duty_timer = TIMEBASE_DUTY_RAMP;
   TMR0_stall_timer = TIMEBASE_STALL_COUNT;
   timebase_10ms = TIMEBASE_LOAD_10ms;
   
   // Timebase manager flags
   TMR0_startup_flag = 0;
   TMR0_warmup_flag = 0;
   TMR0_stall_flag = 0;
   TMR0_slow_start_flag = 0;
   TMR0_duty_flag = 0;
   TMR0_duty_flag = 0;
   slow_start_events = SLOW_STEPS;
   warmup_complete_flag = 0;
   startup_complete_flag = 0;
   slow_start_complete_flag = 0;
   startup_in_progress = 0;
   stop_flag = 0;
   run_flag = 0;

   // TIMER1 
   T1CON = T1CON_INIT;

   // COMPARATORS
   CMxCON0 = CMxCON0_INIT;
   //CMyCON0 = CMyCON0_INIT;

   // DRIVER init
   PR2 = PWM_PERIOD;
   T2CON = T2CON_INIT;
   
   zc_error = 0;
   temp = 0;
   expected_zc.word = 0;
   zc.word = 0;
   comm_after_zc.word = 0;
   ramped_speed = 0;
   //startup_dutycycle = MED_STARTUP_DUTYCYCLE;
      
   init_complete_flag = 0;   
}

/************************************************************************
*                                                                       *
*      Function:       InitDriver                                       *
*                                                                       *
*      Description:     initialize PWM and driver registers & variables *
*                                                                       *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*  Drivers are initialized after the warmup period and when restarting  *
*  after a stop or stall event.                                         *
*                                                                       *
*************************************************************************/

void InitDriver(void) 
{
   // startup duty cycle is set by SupplyManager()
   ramped_speed = FindTableIndex(startup_dutycycle);
   GetCCPVal(ramped_speed);
   CCP1CON = CCP1CON_INIT;        //    PWM on
   PSTR1CON = 0;
#ifdef ECCPAS_INIT
   ECCPAS = ECCPAS_INIT;        // autoshutdown mode
   PWM1CON = PWM1CON_INIT;        // restart mode
#endif

   TMR1_comm_time.word = startup_rpm; //0xFFFF - COMM_TIME_INIT + 1;
   // expected_zc is negative expected time remaining at zero cross event
   expected_zc.word = (TMR1_comm_time.word>>1) | 0x8000;
           
   comm_state=1;
   Commutate();

   startup_in_progress = 1;
   init_complete_flag = 1;   
}

/************************************************************************
*                                                                       *
*      Function:       TimeBaseManager                                  *
*                                                                       *
*      Description:     manage time base timer TIMER0 @ 16.3ms          *
*                                                                       *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*  The TimeBaseManager() is called in the main loop. There are several  *
*  timers in the system which are all controlled by the TimeBaseManager.*
*  The TimeBaseManager() base time resolution is 10mS set by the        * 
*  constant TIMEBASE_LOAD_10ms. All other timers inherit this           *
*  resolution.                                                          *
*  The TimeBaseManager() sets all subroutine timer flags which each     *
*  subroutine then receives as a signal to update their respective      *
*  timers.                                                              *
*                                                                       *
*************************************************************************/
void TimeBaseManager(void) 
{  
	if(T0IF) {
		T0IF = 0;
		TMR0 += TIMEBASE_MANAGER_RELOAD_COUNT;
 
		if(timebase_10ms) {
			timebase_10ms--;
			return;								// wait 10ms
		}

		// 10ms TIMEBASE timer
		//--------------------------------------------------------------------------
		timebase_10ms = TIMEBASE_LOAD_10ms;

      // TMR0 warmup timer update
      TMR0_warmup_flag = 1;
      //if(TMR0_warmup_timer) TMR0_warmup_timer--;

      // TMR0 slow start timer update
      TMR0_slow_start_flag = 1;

      // TMR0 startup timer update
      TMR0_startup_flag = 1;
   
      // TMR0 stall timer update
      TMR0_stall_flag = 1;

      // TMR0 duty ramp timer update
      //    "duty" refers to the PWM duty cycle. The PWM duty cycle
      //    is determined by getting a value from the SpeedManager()
      //    The duty cycle change rate is limited by this
      //    timer which permits one increment/decrement of the duty cycle
      //    register every N mS (set by the TIMEBASE_DUTY_RAMP constant).
      TMR0_duty_flag = 1;
      
   }
}

/************************************************************************
*                                                                       *
*      Function:       WarmUpControl                                    *
*                                                                       *
*      Description:     initialize system after warm up                 *
*                                                                       *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*  Warmup period is controlled by the TimeBaseManager(). Nothing in the *
*  system starts until the warmup period of TIMEBASE_WARMUP_ms has      *
*  elapsed.                                                             *
*                                                                       *
*************************************************************************/

void WarmUpControl(void)
{

   if(warmup_complete_flag) return;     // exit if warmup ended
   if(!(TMR0_warmup_flag)) return;
//    The TMR0_warmup_flag is set every 10mS by the TimeBaseManager().
//    The TMR0_warmup_timer is decremented here every 10mS until,
//    after TIMEBASE_WARMUP_ms, it reaches the value of zero. When the timer reaches
//    zero the drivers are initialized and interrupts are enabled. Additional
//    processing is permantly locked out by the warmup_complete_flag.

   TMR0_warmup_flag = 0;

   if(TMR0_warmup_timer) 
   {
      TMR0_warmup_timer--;
   }
   else
   {

      // the SpeedManager() determines when the run_flag goes true
      // wait until commanded by the speed control before completing warmup
      // and progressing to the next stages of initializing and startup
      if(run_flag)
      {
         warmup_complete_flag = 1;         
         InitDriver();         
      }
   }
}

/************************************************************************
*                                                                       *
*      Function:       ControlSlowStart                                 *
*                                                                       *
*      Description:  Dwell at two successive commutations before        *
*                   starting the spin up                                *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*  This is called every main loop cycle but the TMR0_slow_start_flag    *
*  limits execution to every 10 ms as defined by the TimeBaseManager()  *
*  routine.                                                             *
*                                                                       *
*************************************************************************/

void ControlSlowStart(void)
{
   if(!init_complete_flag) return;           // exit if not initialized
   if(slow_start_complete_flag) return;      // exit if slow start ended
   if(!(TMR0_slow_start_flag)) return;       // hold off for 10 ms
   
   TMR0_slow_start_flag = 0;
   // The slow start timer determines how long to dwell at each slow
   // start commutation point.
   if(--TMR0_slow_start_timer == 0)
   {  // when slow start is complete change to the startup timer
      // which determines how long to ramp-up at fixed commutations
      // the ramp-up is terminated early when the first zero cross
      // is detected.
      if(--slow_start_events == 0)
      {
         //TP0 = 1;  // Diagnostic
         slow_start_complete_flag = 1;
         TMR0_startup_timer = TIMEBASE_STARTUP_COUNT;
         TMR1H = TMR1_comm_time.bytes.high;
         TMR1L = TMR1_comm_time.bytes.low;
         isr_state = commutate;
         TMR1ON = 1;
         TMR1IE = 1;
         PEIE=1;   
         GIE=1;
      }
      else
      {  // reset the dwell timer for the next step
         TMR0_slow_start_timer = TIMEBASE_SLOW_STEP;
         Commutate();
      }
   }
}   

/************************************************************************
*                                                                       *
*      Function:       ControlStartup                                   *
*                                                                       *
*      Description:     check startup flags and timing                  *
*                                                                       *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*  Zero cross must be detected within the middle 25% of the             *
*  commutation period before the startup timer expires otherwise        *
*  the motor will be shut down.                                         *
*                                                                       *
*  Startup time is determined by the constant TIMEBASE_STARTUP_COUNT    *
*  which is defined in BLDC.h. Maximum startup time is 2.55 seconds.    *
*  This is called every main loop cycle but the TMR0_startup_flag limits*  
*  execution to every 10 ms as defined by the TimeBaseManager() routine.*
*                                                                       *
*************************************************************************/

void ControlStartUp(void) 
{
   if(!slow_start_complete_flag) return;     // exit if slow start not ended
   if(!startup_in_progress) return;          // exit if startup ended
   if(!(TMR0_startup_flag)) return;          // hold off for 10 ms
   
   // This flag is set every 10 ms by the TimeBaseManager()
   TMR0_startup_flag = 0;   

   if(--TMR0_startup_timer == 0) 
   {
      // stable rotation will clear the startup_complete_flag.
      // if rotation is not stable by the time the startup is complete
      // then force a full reinitialization by setting the stop_flag..
      startup_in_progress = 0;
      if(!startup_complete_flag)
      {
         stop_flag=1;
      }   
   }
}

/************************************************************************
*                                                                       *
*      Function:       StallControl                                     *
*                                                                       *
*      Description:     check motor stall and re-start                  *
*                                                                       *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/

void StallControl(void) 
{   
   //   unsigned char samplebit;
   
   if(!init_complete_flag) return;      // exit if warmup
   if(!startup_complete_flag) return;
   if(!(TMR0_stall_flag)) return;       // wait TMR0 flag

   TMR0_stall_flag = 0;   

   // TMR0_stall_timer is reset every stable detect event
   // if no stable events are detected then the timer will 
   // time-out forcing the control into a stop condition.
   if(--TMR0_stall_timer == 0)
   {
      stop_flag=1;
   }   
/*   
   // check other stall indicators less often
   if(--TMR0_stallcheck_timer==0)
   {
      TMR0_stallcheck_timer = TIMEBASE_STALLCHECK_COUNT;
      
      // stall is detected when the undriven BEMF voltage is below that of a spinning motor.
      // The measurement is synchronized with the peak BEMF in the middle of a commutation period.
      
      // set a flag to signal the zero-crossing interrupt to capture a BEMF measurement
      // then wait until the capture is performed
      ADCON0 = ADCON0_BEMF;
      measure_bemf_flag = 1;
      while (measure_bemf_flag);
      while(GODONE == 1);
      if(ADRESH < BEMF_LIMIT)
      {
         // motor generator voltge is below the limit
         // if the commmutation rate indicates that the motor should be spinning faster
         // then halt the motor
         if((unsigned int)TMR1_comm_time.word > MAX_TMR1_PRESET)
         {
            stop_flag = 1;
         }   
      }


// Diagnostic to see the ADC voltage read on TP2. 
// Disable other uses of TP2 before enabling this diagonstic
      for(samplebit = 0x80; samplebit != 0; samplebit=samplebit>>1)
      {
         TP2 = 1;
         if ((samplebit & ADRESH) != 0) 
         {
            TP2 = 1;
         }
         else
         {
            TP2 = 0;
         }
      }
      TP2 = 0;    

   }
*/   
}

unsigned char FindTableIndex(unsigned int duty_cycle)
{
   // find the closest table index for the supplied value in the table
   // not very efficient but it's only performed during initialization
   unsigned char index;
   
   for(index = 0; CCP_Values[index]<duty_cycle; index++);
   return index;
}

// This function accepts an 8-bit value as an index for a lookup table.
// The table sets a 10-bit value into the CCPR registers
void GetCCPVal(unsigned char speed)
{     
     CCPR1L = (CCP_Values[speed] >> 2);
     DC1B0  = (CCP_Values[speed] & 0x01)?1:0;
     DC1B1  = ((CCP_Values[speed] >> 1) & 0x01)?1:0;
}

/************************************************************************
*                                                                       *
*     Project              : 3-Phase Brushless Motor Control            *
*                                                                       *
*     Author               : W.R.Brown                                  *
*                        (original program structure: Loris Bianchin)   *
*     Company              : Microchip Technology Incorporated          *
*     Filename             : F1937_Combined_Main.c                      *
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
* w.r.brown            2016.03.11  Revised for MPLAB X
* w.r.brown            2011.03.08  V 1.23  - Changed name of this file and SpeedManager to avoid
*                                            conflicts with other files of same previous name.
* w.r.brown            2011.03.07  V 1.22  - Changed time set method and display mode indicators
* w.r.brown            2009.04.21  V 0.1  - first release
*******************************************************************************************************/
#include <xc.h>
#include "BLDC.h"
#ifdef PC_CONTROL
#include "Monitor.h"
#endif

#include "GenericTypeDefs.h"
#include "lcd.h"
#include "rtcc.h"
#include "input.h"
#include "i2c.h"
#include "mcp9800.h"
#include "mchp_support.h"

void SpeedManager(int);

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
unsigned char TMR0_stallcheck_timer;
unsigned char TMR0_button_held_timer;
volatile unsigned char TMR0_temperature_timer;
unsigned char timebase_10ms;

bit TMR0_startup_flag;
bit TMR0_warmup_flag;
bit TMR0_slow_start_flag;
bit TMR0_stall_flag;
bit TMR0_duty_flag;
bit TMR0_button_flag;
bit TMR0_temperature_flag;
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

char startup_dutycycle;// = MED_STARTUP_DUTYCYCLE;
unsigned int startup_rpm;// = (0xFFFF - COMM_TIME_INIT + 1);

extern enum {high_res_setup,zero_detect,commutate}isr_state;;
extern const int CCP_Values[256];

void display_time(void);
BOOL display_temp(int t);
char display_int(int t);
void display_rpm(int r);
void display_pot(int p);

volatile char blink;

typedef enum { MODE_TIME, MODE_TEMPERATURE, MODE_POT, MODE_RPM } mode_t;
typedef enum { INCR_MINUTES, INCR_TENS, INCR_HOURS } mode_t_incr;

/************************************************************************
*                                                                       *
*                              M A I N                                  *
*                                                                       *
*************************************************************************/
void main(void)
{
    int pot_value;
    event_t btn_event;
    char time_set;
    time_t t;
    mode_t display_mode;
    mode_t_incr time_incr_mode;

	InitSystem();
	
    i2c_init();
    lcd_init();	
    rtcc_init();
    mcp9800_init();

    display_mode = MODE_TIME;
    time_set = 0;
    stop_flag = 1;
#ifdef PC_CONTROL
   MonitorInit();
#endif   

   while(1) {
		CLRWDT();
		if(stop_flag)
		{
			InitSystem();
			i2c_init();
			lcd_init();
		}	 
		TimeBaseManager();
		WarmUpControl();
		ControlSlowStart();
		ControlStartUp();
		StallControl();

        // handle the other tasks
        i2c_handler();
        
        btn_event = input_event();
   	    pot_value = input_pot();

    	switch(btn_event)
    	{
        	case BUTTON_UP: break;
        	case BUTTON_DOWN: // button is sensed as held down
        	    if(display_mode == MODE_TIME && time_set == 0)
        	    {
	        	    if(TMR0_button_flag)
		        	    if(TMR0_button_held_timer-- == 0)
		        	    {
			        	    // enter time set mode when the button is held long enough
		            	    time_set = 1;
		            	    // time set routine will decrement TMR0_button_held_timer to zero
		            	    // and make the first time adjustment immediately
		            	    TMR0_button_held_timer++;
		            	    // read the current time into t and strip off seconds
							time(&t);
							t = t - (t % 60);
							time_incr_mode = INCR_MINUTES;
		            	}
		            else
		            	TMR0_button_flag = 0;
	            }	
        	    break;
        	case BUTTON_PRESSED: // button just became pressed
	        	if(display_mode == MODE_TIME)
	        	{	// set the held-button timer for 0.5 sec (0.5/Timebase manager resolution)
		        	// button must be held down for this amount of time to enter time set mode
	        	    TMR0_button_held_timer = 50;
	        	}    
        	    break;
        	case BUTTON_RELEASED: // on release, next display mode (unless we were setting the time)
            	    if(time_set == 0)
            	    {
	            	    switch(display_mode)
	            	    {
	                	    case MODE_TIME:
	                	        TMR0_temperature_timer = 1;
								     display_mode = MODE_TEMPERATURE;
	                	        break;
	                	    case MODE_TEMPERATURE:
	                	        display_mode = MODE_POT;
	                	        break;
	                	    case MODE_POT:
	                	        display_mode = MODE_RPM; 
	                	        break;
	                	    case MODE_RPM:
	                	    default:
	                	        display_mode = MODE_TIME;
	                	        break;
	                	}
                	}
                	else
                	    // exit time set mode on button release
                	    // time resumes from where it was left
                		time_set = 0;
        	    break;
        	default: break;
        }
        
        // Time set increments the clock time by one minute for every held button interval
        // when the minutes reach an even 10 then the increment is 10 minutes
        // when the minutes reach an even hour then the increment is one hour
        if(time_set) // if we are setting the time ...
        {
            // TimeBaseManager sets the TMR0_button_flag every 10 ms.
	        if(TMR0_button_flag)
	        {
           	    TMR0_button_flag = 0;
        	    if(TMR0_button_held_timer-- == 0)
        	    {
	        	    // increments occur every 1/3 second while the button is held
	        	    TMR0_button_held_timer =  33;
	        	    // adjust the time
	        	    switch (time_incr_mode)
	        	    {
	        	    	case INCR_MINUTES:
		        	    	t += 60;             // increment minutes while not even 10
		        	    	if ((t % 600)==0)    // check even 10
		        	    		time_incr_mode = INCR_TENS;
		        	    	break;
		        	    case INCR_TENS:
	        	    		t += 600;             // increment by 10 minutes while not even hour
		        	    	if ((t % 3600)==0)    // check even 60
		        	    		time_incr_mode = INCR_HOURS;
		        	    	break;
		        	    case INCR_HOURS:
		        	    default:
	        	    		t += 3600;  // increment hour
	        	    		break;
	        	    }
			  		// show the current setting
					rtcc_set(&t);
				}
			}
		}
		else // we are not setting the time
        {
            SpeedManager(pot_value); // use the pot to control the motor speed
            // update the time
            rtcc_handler();
        }
            	 
        switch(display_mode)
        {
            case MODE_TIME: 
                display_time();
                break;
            case MODE_TEMPERATURE:
                // flag is set every 10 ms by TimeBaseManage
                if(TMR0_temperature_flag)
                {
	                TMR0_temperature_flag = 0;
	                // display temperature every .5 sec
	                if(--TMR0_temperature_timer == 0)
	                {
                   		if(display_temp(mcp9800_get_temp()))
                   		    // if display was not inhibited by WA permission then
                   		    // next display will be 500 ms from now
                   			TMR0_temperature_timer = 50;
                   		else
                   		{
                   		    // when display is inhibited by WA then
                   		    // retry next time through loop
                   		    TMR0_temperature_flag = 1;
                   			TMR0_temperature_timer = 1;
                   		}
                   	}
               	}
                break;
            case MODE_RPM:
                display_rpm(GetRPM()/10);
                break;
            case MODE_POT:
                display_pot(pot_value);
                break;
            default: display_mode = MODE_TIME; break;
        }
        
        // Run and loop speed indicator
        RunLED = !RunLED;
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

   // set all ports tristate
   
   TRISA = 0xFF;
   TRISB = 0xFF;
   TRISC = 0xFF;
   
   // initialize ports
   
   PORTA=PORTA_INIT;
   TRISA=TRISA_INIT;

   PORTB=PORTB_INIT;
   TRISB=TRISB_INIT;

   PORTC=PORTC_INIT;
   TRISC=TRISC_INIT;

#ifdef PORTD_INIT
   PORTD=PORTD_INIT;
   TRISD=TRISD_INIT;
#endif
   
#ifdef PORTE_INIT
   PORTE=PORTE_INIT;
   TRISE=TRISE_INIT;
#endif
   
   // initialize ADC
   
   ANSELA=ANSELA_INIT;
   ANSELB=ANSELB_INIT;
   ANSELD=ANSELD_INIT;
   ANSELE=ANSELE_INIT;

   ADCON0 = ADCON0_INIT;
   ADCON1 = ADCON1_INIT;
   
   CCP1CON = 0;            //disable PWM

   // TIMER0 and related startup variables
   TMR0 = 0;
   T0IF = 0;
   OPTION_REG = OPTION_REG_INIT;
   TMR0_warmup_timer = TIMEBASE_WARMUP_COUNT;
   TMR0_slow_start_timer = TIMEBASE_SLOW_STEP;
   TMR0_duty_timer = TIMEBASE_DUTY_RAMP;
   TMR0_stallcheck_timer = TIMEBASE_STALLCHECK_COUNT;
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
   startup_dutycycle = MED_STARTUP_DUTYCYCLE;
   startup_rpm = (0xFFFF - COMM_TIME_INIT + 1);
   
      
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
   // Convert ECCP duty cycle number to the index number
   // The index number is the 8-bit speed request value
   ramped_speed = FindTableIndex(startup_dutycycle);
   // GetCCPValue actually gets the ECCP duty cycle from the table
   // and sets the ECCP duty cycle registers 
   GetCCPVal(ramped_speed);
   CCP1CON = CCP1CON_INIT;        //    PWM on
#ifdef ECCPAS_INIT
   ECCPAS = ECCPAS_INIT;        // autoshutdown mode
   PWM1CON = PWM1CON_INIT;        // restart mode
#endif

   TMR1_comm_time.word = startup_rpm; //0xFFFF - COMM_TIME_INIT + 1;
   
   // expected_zc is negative expected time remaining at zero cross event
   expected_zc.word = (TMR1_comm_time.word>>1) | 0x8000;
           
   comm_state=2;
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
      
      // Measure the time the pushbutton is held
      TMR0_button_flag = 1;
      TMR0_temperature_flag = 1;
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
*      Function:       GetRPM                                           *
*                                                                       *
*      Description:     calculate the current motor RPM                 *
*                                                                       *
*      Parameters:                                                      *
*      Return value:    RPM                                             *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/

unsigned long GetRPM(void)
{
    unsigned long rpm;
    
    if(!run_flag) // the motor is not running, return 0
        return 0;
        
    rpm = 60L * 1000L * 1000L;
    rpm /= NUM_POLES * NUM_PHASES * (65535 - (unsigned int)TMR1_comm_time.word);
    
    return rpm;
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

   // check other stall indicators less often
   if(--TMR0_stallcheck_timer==0)
   {
      TMR0_stallcheck_timer = TIMEBASE_STALLCHECK_COUNT;
      
     // if the commmutation rate indicates that the motor should be spinning faster
     // then halt the motor
     if((unsigned int)TMR1_comm_time.word > MAX_TMR1_PRESET)
     {
        stop_flag = 1;
     }   
   }
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
     DC1B0  = CCP_Values[speed] & 0x01;
     DC1B1  = (CCP_Values[speed] >> 1) & 0x01;
}


/*************************************/
/* LCD HELPERS                       */
/*************************************/
BOOL display_temp(int t)
{
	/*
    static int counter = 0;
    static int integrator=0;
    int average;
    static int v = 0;
    
    // low pass filter
    integrator += t;
    average = integrator / 16;
    integrator -= average;
    
    // decimate
    if(counter-- == 0)
    {
        counter = 100;
        v = average;
    }

    // display
    display_int(v);
    */
    if(display_int(t))
    {
	    DP3 = 0;
	    DP2 = 1;
	    S1 = 0;
	    S2 = 0;
	    AMPS = 0;
	    VOLT = 0;
	    KILO = 1;
	    OHMS = 0;
       MINUS = 0;
	    DH = 0;
	    RH = 0;
	    RC = 0;
	    return (1);
    }
    return (0);
}

void display_temp_heavyfilter(int t)
{
    static int counter = 0;
    static long integrator=0;
    static long integrator2=0;
    int average;
    int average2;
    static int v = 0;
    
    // low pass filter
    integrator += t;
    average = integrator / 32;
    integrator -= average;
    integrator2 += average;
    average2 = integrator2 / 7;
    integrator2 -= average2;
    
    // decimate & damp display
    if(counter-- == 0)
    {
        counter = 300;
        if(v + 10 < average2) // we are way below the average
        {
            v += 10;
        }
        else if (v - 10 > average2) // we are way above the average
        {
            v -= 10;
        }
        else if(v > average2)
        {
            v --;
        }
        else if(v < average2)
        {
            v ++;
        }
        else
        {
            // do nothing v == average2
        }        
    }

    // display
    if(display_int(v))
    {
        DP3 = 0;
        DP2 = 1;
        S1 = 0;
        S2 = 0;
        AMPS = 0;
        VOLT = 0;
        KILO = 0;
        OHMS = 0;
        MINUS = 0;
        RH = 1;
        DH = 0;
        RC = 0;
    }
}
    
void display_rpm(int r)
{
    static int counter = 0;
    static int integrator=0;
    int average;
    static int v = 0;
    
    // low pass filter
    integrator += r;
    average = integrator / 16;
    integrator -= average;
    
    // decimate
    if(counter-- == 0)
    {
        counter = 100;
        v = average;
    }

    // display
    if(display_int(v))

    {
        DP3 = 0;
        DP2 = 0;
        S1 = 0;
        S2 = 0;
        AMPS = 0;
        VOLT = 0;
        KILO = 0;
        OHMS = 0;
        MINUS = 0;
        DH = 0;
        RH = 1;
        RC = 0;
    }    
}
    
void display_pot(int p)
{
    if(display_int(p))
    {
        DP3 = 0;
        DP2 = 0;
        S1 = 0;
        S2 = 0;
        AMPS = 0;
        VOLT = 0;
        KILO = 0;
        OHMS = 0;
        MINUS = 0;
        DH = 1;
        RH = 0;
        RC = 0;
    }    
}

char display_int(int t)
{
    BCD_TYPE bcd;
    
    // assumes t is the temperature in degrees C * 10
    bcd.digit0 = t %10;
    t /= 10;
    bcd.digit1 = t % 10;
    t /= 10;
    bcd.digit2 = t % 10;
    t /= 10;
    bcd.digit3 = t%10;
    
    return (lcd_display_digits(bcd));   
}    

void display_time(void)
{
    time_t t;
    struct tm *d;
    BCD_TYPE bcd;
    
    time(&t);
    d = gmtime(&t);
    
    bcd.digit0= d->tm_min % 10;
    bcd.digit1= d->tm_min / 10;
    
    if(d->tm_hour > 11)
    {
        d->tm_hour -= 12;
        AMPS = 0;
    }    
    else
    {
        AMPS = 1;  // AM indicator
    }
    
    if(d->tm_hour == 0) d->tm_hour = 12;
    
    bcd.digit2= d->tm_hour % 10;
    bcd.digit3= d->tm_hour/10;

    if(lcd_display_digits(bcd))
    {
        if((t % 2) == 1)
        {
            DP3 = 1;
        }    
        else
        {
            DP3 = 0;
        }    
        DP2 = 0;
        S1 = 0;
        //AMPS = 0;
        VOLT = 0;
        KILO = 0;
        OHMS = 0;
        RC = 1;
        DH = 0;
        RH = 0;
    }    
}

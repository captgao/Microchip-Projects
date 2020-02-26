/************************************************************************
*                                                                       *
*     Project              : 3-Phase Brushless Motor Control            *
*                                                                       *
*     Author               : w.r.brown                                  *
*                                                                       *
*     Company              : Microchip Technology Incorporated          *
*     Filename             : BLDC.h                                     *
*     Date                 : 2009/04/14                                 *
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
// Common header file for all Enhanced mid-range core BLDC builds.                                    //
// Define the motor and driver combination in the project macro defines                               //
// Example: Project->Build Options...->Project->Compiler                                              //
// This file sorts out the macro defines to determine the proper motor and driver                     //
// header files to include in the build.                                                              //
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
* w.r.brown            2016.03.11  Revised for MPLAB X
* w.r.brown            2009.07.28  V 0.2 - Corrected TIMEBASE_MANAGER_RELOAD_COUNT
*                                          Moved Timer 0 UART break timing definitions to UartInterface.c
* w.r.brown            2009.03.25  V 0.1 - Derived from PIC16F882 BLDC.h
*
*******************************************************************************************************/

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// MCU Constants ///////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
// SYSTEM OSCILLATOR -
//
// Pick one: 8 MHz or 32 MHz
//#define  FOSC_8_MHZ
#define  FOSC_32_MHZ

#ifdef   FOSC_32_MHZ
//// Primary oscillator
#define  FOSC                    32000000L
#define  OSCCON_INIT             0b11110000
// Timer0: 1:64 prescale
#define OPTION_REG_INIT          0b10000101
#define TIMER0_PRESCALE          64L
// Timer1: prescaler 1:8 = 1 us period @32MHz
#define  T1CON_INIT              0b00110001
#define  TMR1_PRESCALE           8L
 //Timer2: on, 1:4 prescale, 1:1 postscale, 0.5 us period @32MHz
#define TIMER2_PRESCALE          4L
#define T2CON_INIT               0b00000101
#endif

#ifdef   FOSC_8_MHZ
// Primary oscillator
#define  FOSC                    8000000L
#define  OSCCON_INIT             0b01110000
// Timer0: 1:16 TMR0 prescale
#define OPTION_REG_INIT          0b10000011
#define TIMER0_PRESCALE          16L
// Timer1: prescaler 1:2 = 1 us period @8MHz
#define  T1CON_INIT              0b00010001
#define  TMR1_PRESCALE           2L
// Timer2: on, 1:1 prescale, 1:1 postscale, 0.5 us period @8MHz
#define TIMER2_PRESCALE          1L
#define T2CON_INIT               0b00000100
#endif

#define  SYSTEM_FREQUENCY        (FOSC/4L)

//////////////////////////////////////////////////////////////////////////////////////////
// Watch Dog Timer
//
// Prescale = 1:4096 resulting in WDT period of approximately 132 ms 
#define  SWDTCON_INIT            0b00001110

//////////////////////////////////////////////////////////////////////////////////////////
// TIMER0 based
// Timer0 is used for the TimebaseManager() routine which determines how often each
// service, other than motor control, is perfomed.
// Services include: Supply monitor, Speed request monitor, warmup time, 
//                   slow start step interval, and stall check
//----------------------------------------------------------------------------------------------
#ifndef T0IF
#define T0IF TMR0IF
#endif

#define MILLISECONDS_PER_SEC              1000L

// number of milliseconds in each timebase count
// (i.e. resolution of TimebaseManager() in milliseconds)
#define TIMEBASE_MS_PER_COUNT             10L

// number of milliseconds in each timebase manager tick clock
#define TIMEBASE_MS_PER_TICK              2L
#define TIMEBASE_LOAD_10ms                (TIMEBASE_MS_PER_COUNT/TIMEBASE_MS_PER_TICK)

// timebase reload value for TimebaseManager() interrupt period: 
#define TIMEBASE_MANAGER_PERIOD_COUNT     (TIMEBASE_MS_PER_TICK*SYSTEM_FREQUENCY)/(TIMER0_PRESCALE*MILLISECONDS_PER_SEC)

#define TIMEBASE_MANAGER_RELOAD_COUNT     (0x00FF - TIMEBASE_MANAGER_PERIOD_COUNT)

#if (TIMEBASE_MANAGER_PERIOD_COUNT > 0x00FF)
   #error "TIMER0_PRESCALE is too small. Timebase reload count exceeds 0xFF." 
#endif

#if (TIMEBASE_MANAGER_PERIOD_COUNT < 0x80)
   #warning "TIMER0_PRESCALE can be decreased to improve timebase resolution"
#endif
//////////////////////////////////////////////////////////////////////////////////////////
/////////////////////// TIMEBASE times converted to counts ///////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
#define TIMEBASE_STARTUP_COUNT            (TIMEBASE_STARTUP_ms/TIMEBASE_MS_PER_COUNT)
#define TIMEBASE_STALL_COUNT              (TIMER0_STALL_ms/TIMEBASE_MS_PER_COUNT)
#define TIMEBASE_WARMUP_COUNT             (TIMEBASE_WARMUP_ms/TIMEBASE_MS_PER_COUNT)
#define TIMEBASE_SUPPLY_COUNT             (TIMEBASE_SUPPLY_ms/TIMEBASE_MS_PER_COUNT)
#define TIMEBASE_STALLCHECK_COUNT         (TIMEBASE_STALLCHECK_ms/TIMEBASE_MS_PER_COUNT)

//////////////////////////////////////////////////////////////////////////////////////////
// TIMER1 based
//----------------------------------------------------------------------------------------------

// TIMER1

#define TMR1_FREQUENCY              (SYSTEM_FREQUENCY/TMR1_PRESCALE)
#define MICROSECONDS_PER_SECOND     1000000L
#define TMR1_COUNTS_PER_us          (TMR1_FREQUENCY/MICROSECONDS_PER_SECOND)
#define TMR1_COUNTS_PER_SEC         TMR1_FREQUENCY

#define TIMER1_HIGH_GATE_COUNT      0xff

// overhead adjusts for the time lost when reloading the timer
#define TIMER1_OVERHEAD             (12/TIMER1_PRESCALE)

// timer1 load values based on blanking count
#define TIMER1_HIGH_BLANKING_COUNT   (0xFF - (BLANKING_COUNT_us/256L)) 
#define TIMER1_LOW_BLANKING_COUNT   ((0xFFFF - BLANKING_COUNT_us) & 0xFF)

// dwell time allowing flyback currents to settle before BEMF voltage check
#define BEMF_DELAY_COUNT            (BEMF_ACQUISITION_us * TMR1_COUNTS_PER_us)
#define ACQUISITION_DELAY_COUNT     (ACQUISITION_TIME_us * TMR1_COUNTS_PER_us)

//////////////////////////////////////////////////////////////////////////////////////////
// PWM and TIMER2

// PWM mode
#define CCP1CON_INIT           0b00001100

#define TIMER2_FREQUENCY       (SYSTEM_FREQUENCY/TIMER2_PRESCALE)

// 1/16KHz = 62.5µS PWM period @ 0.5 us Timer2 period
#define PWM_FREQUENCY          16000L
#define PWM_PERIOD             (((TIMER2_FREQUENCY/PWM_FREQUENCY)-1L)&0xFF)   

 // Stall event trigger
#define MAX_TMR1_PRESET        (0xFFFF - MIN_COMM_TIME)


//////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// Prototypes ////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
typedef __bit bit;
void __interrupt() interrupt_handler(void);

void InitSystem(void);
void InitDriver(void);
void Commutate(void);
void ControlStartUp(void);
void StallControl(void);
void WarmUpControl(void);
void TimeBaseManager(void);
void ControlSlowStart(void);
void GetCCPVal(unsigned char speed);
unsigned char FindTableIndex(unsigned int duty_cycle);
unsigned long GetRPM(void);

/////////////////////////////////////////////////////////////////////////////
// Unions and structures
/////////////////////////////////////////////////////////////////////////////

typedef union {
   int word;
   struct {
      char low;
      char high;
   }bytes;   
} doublebyte;

/////////////////////////////////////////////////////////////////////////////
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
/////////////////////////////////////////////////////////////////////////////
//                                                                         //
// NOTE: The following drivers and motor definitions are defined in        //
//       the project "ProjectProperties" dialog. To access the dialog      //
//       use pull down menu File->ProjectProperties then in the Categories // 
//       frame select CONF:[default]->XC8GlobalOptions->XC8Compiler        //
//       Enter the desired driver board and motor type on the Define       //
//       macros line of the Processing and messages category in the        //
//       Options frame.                                                    //
//                                                                         //
/////////////////////////////////////////////////////////////////////////////
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// Driver boards
/////////////////////////////////////////////////////////////////////////////
#ifdef VIRTUAL_HALL_DRIVER
   // 20-pin carrier version
   #include "Virtual_BLDC.h"
#endif

#ifdef JANSO_DRIVER
   #include "Janso_Driver.h"
#endif

#ifdef F1937_DRIVER
   #include "1937_DRIVER.h"
#endif

/////////////////////////////////////////////////////////////////////////////
// Motor types
/////////////////////////////////////////////////////////////////////////////

#ifdef MSE1_MOTOR
   #include "MSE1_Motor.h"
#endif

#ifdef AMETEK
   #include "Ametek.h"
#endif

#ifdef EBM_MOTOR
   #include "EBM_Motor.h"
#endif

/************************************************************************
*                                                                       *
*     Project              : 3-Phase Brushless Motor Control            *
*                                                                       *
*     Author               : w.r.brown                                  *
*                                                                       *
*     Company              : Microchip Technology Incorporated          *
*     Filename             : 1937_Driver.h                              *
*     Date                 : 2009/04/14                                 *
*     SharePoint Version   : 0.2                                        *
*                                                                       *
*     Other Files Required :                                            *
*     Tools Used: MPLAB GL : 8.30                                       *
*                 Compiler : Hi-Tech                                    *
*                 Assembler:                                            *
*                 Linker   :                                            *
*                                                                       *
*************************************************************************
*
////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                    //
// I/O definitions for PIC16F1937 starter board controller                                            //
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
* w.r.brown            2009.07.03  V 0.2  - Modified for PIC16F1937
* w.r.brown            2009.04.14  V 0.1  - Fresh start with PIC16F882
*
*******************************************************************************************************/

#ifndef P1937_DRIVER_H
#define P1937_DRIVER_H

//////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// User Constants ///////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

#ifndef GODONE
#define GODONE GO_nDONE
#endif

//////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// TimebaseManager times ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

// all TIMEBASE times must be expressed in even multiples of TIMEBASE_MS_PER_COUNT
// See BLDC.h

// TIMEBASE_STARTUP_ms = number of milliseconds allowed to achieve zero-cross lock
// (maximum ms is 255*TIMEBASE_MS_PER_COUNT)
#define  TIMEBASE_STARTUP_ms           2000

// time for continuous unstable condition to cause stall
// TIMER0_STALL_ms = number of milliseconds in out-of-lock condition to recognize stall
// (maximum ms is 255*TIMEBASE_MS_PER_COUNT)
#define  TIMER0_STALL_ms               1000
                                          
// number of milliseconds to dwell in warmup state
// (maximum ms is 255*TIMEBASE_MS_PER_COUNT)
#define  TIMEBASE_WARMUP_ms            400

// number of milliseconds between each motor supply voltage check
// (maximum ms is 255*TIMEBASE_MS_PER_COUNT)
#define  TIMEBASE_SUPPLY_ms            50

// number of milliseconds between each motor stall condition check
// (maximum ms is 255*TIMEBASE_MS_PER_COUNT)
#define  TIMEBASE_STALLCHECK_ms        500


//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////// Stall detection BEMF voltage  ///////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

// R1 and R2 are series resistors from each motor terminal to ground
// ADC is connected to the junction of R1 and R2. The other end of R2 is connected to ground.
// Specify VDD_SUPPLY, BEMF_R1, and BEMF_R1 as 10 times the actual values
// Resistors are in K Ohms. Example: 5.6K is 56L.
#define  VDD_SUPPLY                 50L
#define  BEMF_R1                    470L
#define  BEMF_R2                    100L

// 10 times the BEMF voltage below which the motor will assumed to be stalled
// (i.e. for 1.2 Volts define limit as 12L)
#define  MINIMUM_BEMF_VOLTAGE       20L

#define  BEMF_LIMIT                 ((255L*BEMF_R2*MINIMUM_BEMF_VOLTAGE)/(VDD_SUPPLY*(BEMF_R1+BEMF_R2)))

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////// Driver Board Configuration //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

// HIGH_SIDE_MODULATION - Define this variable for systems that modulate the high side driver.
//                        Comment out HIGH_SIDE_MODULATION so it remains undefined for systems 
//                        that modulate the low side driver.
//#define  HIGH_SIDE_MODULATION

// Device PIC16F1937 TQFP Pin assignments:
// 
// Pin #   PORT ID I/O Use  Name    Description
//------   ------- --- ---  ------- --------------------------
//  19       RA0    I      (C12IN0-) LCD Seg12 (or Current sense w/o LCD)
//  20       RA1    I  AN1  C12IN1- BEMF A 
//  21       RA2    I      (C2IN+)  LCD COM3   (or Current sense reference w/o LCD)
//  22       RA3    I  AN3  C1IN+   BEMF Reference
//  23       RA4    I      (C1OUT)  LCD Seg4   (or ZC Test point w/o LCD)
//  24       RA5    I      (C2OUT)  LCD Seg5   (or Overcurrent test point w/o LCD)
//  31       RA6    I       RA6     LCD Seg1
//  30       RA7    I       RA7     LCD Seg2
//   8       RB0    I       RB0/INT LCD Seg0
//   9       RB1    I  AN10 C12IN3- BEMF C
//  10       RB2    I  AN8  RB2     Speed control 
//  11       RB3    I  AN9  C12IN2- BEMF B
//  14       RB4    I       RB4     LCD COM1
//  15       RB5    I       RB5     LCD COM2
//  16       RB6    O       PGC     ICSP Clock (TestPoint 2)
//  17       RB7    O       PGD     ICSP Data  (TestPoint 0)
//  32       RC0    O       RC0     (TestPoint 1)
//  35       RC1    O       RC1     unused
//  36       RC2    O       P1A     Low Side Drive A  
//  37       RC3    O       RC3     unused
//  42       RC4    O       RC4     unused
//  43       RC5    I       RC5     LCD Seg10
//  44       RC6    O       TX      Serial Transmit
//   1       RC7    I       RX      Serial Receive
//  38       RD0    I       RD0     LCD COM4
//  39       RD1    O       RD1     LED indicator
//  40       RD2    I       RD2     Push button
//  41       RD3    I       RD3     LCD Seg16
//   2       RD4    I       RD4     LCD Seg17
//   3       RD5    O       P1B     Low Side Drive B
//   4       RD6    O       P1C     Low Side Drive C
//   5       RD7    I       RD7     LCD Seg20
//  25       RE0    O       RE0     High Side Drive A
//  26       RE1    O       RE1     High Side Drive B
//  27       RE3    O       RE2     High Side Drive C
//  18       RE3    I       Vpp     MCLR
//   7                      Vdd     Power supply
//  28                      Vdd     Power supply
//   6                      Vss     Supply return
//  29                      Vss     Supply return
//
//////////////////////////////////////////////////////////////////////////////////////////
// PORTA (PORT) 
#define   TRISA_INIT       0b11111111
#define   PORTA_INIT       0b00000000
#define   TRISA_ERROR      0b11111111
#define   PORTA_ERROR      0b11111111

//////////////////////////////////////////////////////////////////////////////////////////
// PORTB (PORT)
#define   TRISB_INIT       0b00111111
#define   PORTB_INIT       0b00000000
#define   TRISB_ERROR      0b11111111
#define   PORTB_ERROR      0b11111111
   
//////////////////////////////////////////////////////////////////////////////////////////
// PORTC (PORT)
#define   TRISC_INIT       0b10100000
#define   PORTC_INIT       0b00000000
#define   TRISC_ERROR      0b11111111
#define   PORTC_ERROR      0b11111111

//////////////////////////////////////////////////////////////////////////////////////////
// PORTD (PORT)
#define   TRISD_INIT       0b10011100
#define   PORTD_INIT       0b00000000
#define   TRISD_ERROR      0b11111111
#define   PORTD_ERROR      0b11111111

//////////////////////////////////////////////////////////////////////////////////////////
// PORTE (PORT)
#define   TRISE_INIT       0b00000000
#define   PORTE_INIT       0b00000000
#define   TRISE_ERROR      0b11111111
#define   PORTE_ERROR      0b11111111

//////////////////////////////////////////////////////////////////////////////////////////
// COMPARATORS

// redefine comparator definitions to ease selection between C1 and C2
// CMx is used for BEMF sense
// CMy would have been used for current sense except for the LCD
#define CMxCON0      CM1CON0
#define CMxCON1      CM1CON1
#define CxIE         C1IE
#define CxIF         C1IF
#define CxOUT        MC1OUT
#define COMPARATOR   CMxCON1

#define CMyCON0      CM2CON0
#define CMyCON1      CM2CON1
#define CyIE         C2IE
#define CyIF         C2IF
#define CyOUT        MC2OUT

//////////////////////////////////////////////////////////////////////////////////////////
// Comparator bit definitions
// 
// CMxCON0
#define CxON            0b10000000
#define CxOE            0b00100000
#define CxINV           0b00010000
#define CxFAST          0b00000100
#define CxHYST          0b00000010
#define CxSYNC          0b00000001

// CMxCON1
#define CxINTP          0b10000000
#define CxINTN          0b01000000
#define CxPIN           0b00000000
#define CxCDAC          0b00010000  
#define CxFVR           0b00100000
#define CxGND           0b00110000
#define CxIN0           0b00000000
#define CxIN1           0b00000001
#define CxIN2           0b00000010
#define CxIN3           0b00000011

//////////////////////////////////////////////////////////////////////////////////////////
// Comparator initializations

// BEMF comparator initialization
#ifdef HIGH_SIDE_MODULATION
// comparator ouput is inverted for high side modulation
#define  CMxCON0_INIT         CxON | CxOE | CxFAST | CxINV
#else
#define  CMxCON0_INIT         CxON | CxFAST
#endif

#define  CMxCON1_INIT         SENSE_V_RISING

// Overcurrent sense comparator initialization (can't use because of LCD)
//#define CMyCON0_INIT    CxON | CxOE | CxINV  | CxPIN | CxIN3     

//////////////////////////////////////////////////////////////////////////////////////////
// ADC

#define  ANSELA_INIT          0b00001010
#define  ANSELB_INIT          0b00001110
#define  ANSELD_INIT          0b00000000
#define  ANSELE_INIT          0b00000000

//  read AN8 for speed control
#define  ADCON0_SPEED         0b00100001

// initialize in speed control mode
#define  ADCON0_INIT          ADCON0_SPEED   

#ifdef   FOSC_32_MHZ
//  TAD clock = Fosc/64 (2uS @ 32 MHz), Left justified, Vref-:Vss, Vref+:Vdd
#define  ADCON1_INIT          0b01100000
#endif

#ifdef   FOSC_8_MHZ
//  TAD clock = Fosc/16 (2uS @ 8 MHz)
#define  ADCON1_INIT          0b01010000
#endif

// ACQUISITION_TIME is the time between setting the GODONE bit and conversion start 
#define  ACQUISITION_TIME_us  4L

//////////////////////////////////////////////////////////////////////////////////////////
// ECCP

// Auto-shutdown precluded by LCD interference with Comparator
// auto shutdown on C2OUT, drive output pins to 0
//#define  CCP1AS_INIT         0b00100000

// auto restart
//#define  PWM1CON_INIT         0b10000000

//////////////////////////////////////////////////////////////////////////////////////////
// Driver and comparator sense states

// Phase               A        B        C
// PWM Side Drive   P1A/RC2  P1B/RD5  P1C/D6
// Fixed Side Drive   RE0      RE1     RE2
// BEMF Sense       C12IN1-  C12IN2-  C12IN3-

#ifndef PSTRCON
#define PSTRCON PSTR1CON
#endif

// Direct drive writes the port in lieu of setting/clearing bits
// to minimize dead time between changes
#define DRIVE_PORT           LATE
#define DRIVE_INIT           0b00000000

// PORT control of drive pins
#define DRIVE_A              0b00000001
#define DRIVE_B              0b00000010
#define DRIVE_C              0b00000100
#define DRIVE_U              DRIVE_A
#define DRIVE_V              DRIVE_B
#define DRIVE_W              DRIVE_C

// PWM Drive steering
#define MODULATE_U_BIT       STR1A
#define MODULATE_V_BIT       STR1B
#define MODULATE_W_BIT       STR1C

#define MODULATE_A           0b00000001
#define MODULATE_B           0b00000010
#define MODULATE_C           0b00000100
#define MODULATE_U           MODULATE_A
#define MODULATE_V           MODULATE_B
#define MODULATE_W           MODULATE_C

#define SENSE_A_RISING       CxINTP | CxPIN | CxIN0
#define SENSE_B_RISING       CxINTP | CxPIN | CxIN1
#define SENSE_C_RISING       CxINTP | CxPIN | CxIN2
#define SENSE_U_RISING       SENSE_A_RISING
#define SENSE_V_RISING       SENSE_B_RISING
#define SENSE_W_RISING       SENSE_C_RISING

#define SENSE_A               SENSE_A_RISING
#define SENSE_B               SENSE_B_RISING
#define SENSE_C               SENSE_C_RISING
#define SENSE_U               SENSE_A_RISING
#define SENSE_V               SENSE_B_RISING
#define SENSE_W               SENSE_C_RISING

#define SENSE_A_FALLING      CxINTP | CxPIN | CxIN0
#define SENSE_B_FALLING      CxINTP | CxPIN | CxIN1
#define SENSE_C_FALLING      CxINTP | CxPIN | CxIN2
#define SENSE_U_FALLING      SENSE_A_FALLING
#define SENSE_V_FALLING      SENSE_B_FALLING
#define SENSE_W_FALLING      SENSE_C_FALLING

// HIGH_SIDE_MODULATION is determined in at the top of this file.
#ifdef HIGH_SIDE_MODULATION
   // In high side modulation systems the BEMF is sensed as BEMF rises
   #define BEMF_FLAG rising_bemf_flag
#else
   // In low side modulation systems the BEMF is sensed as BEMF falls
   #define BEMF_FLAG !rising_bemf_flag
#endif

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////// Test Point Definitions //////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

// use these points for scope reference and triggering

// startup and stable
#define TP0                  LATB7

// comm state 2 to end of blanking
#define TP1                  LATC0

// zero detect
#define TP2                  LATB6

// commutate to blanking
//#define TP3                  RC3

// run indicator
//#define RunLED                LATD1

#define SW2                   RD2

#endif

#define AIN1_SEL ANSA0
#define AIN1_CH 0
#define AIN3_SEL ANSA1
#define AIN3_CH 1
#define AIN4_SEL ANSA3
#define AIN4_CH 3
#define AIN5_SEL ANSB3
#define AIN5_CH 9
#define AIN7_SEL ANSA5
#define AIN7_CH 5
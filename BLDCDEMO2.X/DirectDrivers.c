/************************************************************************
*                                                                       *
*     Project              : 3-Phase Brushless Motor Control            *
*                                                                       *
*     Author               : W.R.Brown                                  *
*                                                                       *
*     Company              : Microchip Technology Incorporated          *
*     Filename             : DirectDrivers.c                            *
*     Date                 : 2008/03/21                                 *
*     SharePoint Version   : 1.0                                        *
*                                                                       *
*     Tools Used: MPLAB GL : 8.01                                       *
*                 Compiler : Hi-Tech                                    *
*                 Assembler:                                            *
*                 Linker   :                                            *
*                                                                       *
************************************************************************/

////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                    //
// Discrete FET driver commutation states.                                                            //
//                                                                                                    //
////////////////////////////////////////////////////////////////////////////////////////////////////////


/*******************************************************************************************************
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
* w.r.brown            2009.03.12  V 1.0  - Added header information
*******************************************************************************************************/

#include <xc.h>
#include "BLDC.h"
#include "EBM_Motor.h"
#include "1937_DRIVER.h"
typedef void (*GateHandler)(void);
typedef struct {
    GateHandler drive[6];
} GateState_t;
void state_drive(unsigned char state);
void set_cmp(unsigned char state);
extern unsigned char comm_state; 
extern bit rising_bemf_flag;

/************************************************************************
*                                                                       *
*      Function:       Commutate                                        *
*                                                                       *
*      Description:    driver commutation                               *
*                                                                       *
*      PSTRCON is used to steer PWM to one side of bridge and           *
*      DRIVE_PORT is used to drive other side. Comparator input         *
*      is steered to undriven phase of motor.                           *
*                                                                       *
*      Parameters:                                                      *
*      Return value:                                                    *
*                                                                       *
*************************************************************************/

//
//    4|5|6|1|2|3|4|5|6|1|2|3|
//     | | |_|_| | | | |_|_| |
//  U _|_|/| | |\|_|_|/| | |\|
//    _| | | | |_|_| | | | |_|
//  V  |\|_|_|/| | |\|_|_|/| |
//     |_|_| | | | |_|_| | | |
//  W /| | |\|_|_|/| | |\|_|_|
//
// State  Low   High  Comparator
//   1     V     U       -W
//   2     W     U        V
//   3     W     V       -U
//   4     U     V        W
//   5     U     W       -V
//   6     V     W        U
//
void Commutate(void)
{
    if(comm_state == 0) return;
    else if(comm_state == 6) {
        comm_state = 1;
    }
    else comm_state++;
    state_drive(comm_state);
    set_cmp(comm_state);
    
} // end Commutate
 void set_cmp(unsigned char state) {
    switch (state) {
        case(1):
        {
            //  W high to low
            COMPARATOR=SENSE_W_FALLING;
            rising_bemf_flag = 0;
            break;
        }
        case(2):
        {
            // V low to high  
            COMPARATOR=SENSE_V_RISING;
            rising_bemf_flag = 1;
            break;
        }
        case(3):
        {
            //U high to low
            COMPARATOR=SENSE_U_FALLING;
            rising_bemf_flag = 0;
            break;
        }
        case(4):
        {
            // W low to high
            COMPARATOR=SENSE_W_RISING;
            rising_bemf_flag = 1;
            break;
            
        }
        case(5):
        {
            //V high to low
            COMPARATOR=SENSE_V_FALLING;
            rising_bemf_flag = 0;
            break;
        }
        case(6):
        {
            COMPARATOR=SENSE_U_RISING;
            rising_bemf_flag = 1;
            break;
        }
        default:break;
    }
}
#define PWM_OFF_UL STR1A=0
#define PWM_ON_UL  STR1A=1
#define PWM_OFF_VL STR1B=0
#define PWM_ON_VL STR1B=1
#define PWM_OFF_WL STR1C=0
#define PWM_ON_WL STR1C=1
#define PWMH_U LATC5
#define PWMH_V LATC1
#define PWMH_W LATC0
void UHoff() {
    PWMH_U = 0;
}
void UHon(){
    PWMH_U = 1;
}
void ULoff(){
    PWM_OFF_UL;
}
void ULon(){
    PWM_ON_UL;
}
void VHoff(){
    PWMH_V = 0;
}
void VHon(){
    PWMH_V = 1;
}
void VLoff(){
    PWM_OFF_VL;
}
void VLon(){
    PWM_ON_VL;
}
void WHoff(){
    PWMH_W = 0;
}
void WHon(){
    PWMH_W = 1;
}
void WLoff(){
    PWM_OFF_WL;
}
void WLon(){
    PWM_ON_WL;
}

static const GateState_t gateStates[] = {
    {UHoff, ULoff, VHoff, VLoff, WHoff, WLoff},
    {ULoff, VHoff, WHoff, WLoff, UHon, VLon},
    {ULoff, VHoff, VLoff, WHoff, UHon, WLon},
    {UHoff, ULoff, VLoff, WHoff, VHon, WLon},
    {UHoff, VLoff, WHoff, WLoff, VHon, ULon},
    {UHoff, VHoff, VLoff, WLoff, WHon, ULon},
    {UHoff, ULoff, VHoff, WLoff, WHon, VLon},
};
void state_drive(unsigned char state) {
    gateStates[state].drive[0]();
    gateStates[state].drive[1]();
    gateStates[state].drive[2]();
    gateStates[state].drive[3]();
    gateStates[state].drive[4]();
    gateStates[state].drive[5]();
}
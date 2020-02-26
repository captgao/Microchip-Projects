/************************************************************************
*                                                                       *
*     Project              : 3-Phase Brushless Motor Control            *
*                                                                       *
*     Author               : Frank Hammerschmidt                        *
*                                                                       *
*     Company              : Microchip Technology Incorporated          *
*     Filename             : LinearSpeedProfile_16K.c                   *
*     Date                 : 2009/03/11                                 *
*     SharePoint Version   : 1.1                                        *
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
//   Speed lookup tables for 16 kHz PWM frequency. Linear voltage output.                             //
//                                                                                                    //
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
* w.r.brown            2009.03.24  V 1.1  - Added 10-bit table for 64 MHz clock
* w.r.brown            2009.03.15  V 1.0  - First release
* w.r.brown            2009.03.14  V 0.1  - Built from TF40NaturalSpeedProfile.c
*                                           Deleted second table and changed table name
*******************************************************************************************************/

#include "BLDC.h"

//***********************************************************************
// Lookup Tables 
// 10-bit CCP Values used for PWM motor control

#ifdef FOSC_64_MHZ

const int CCP_Values[256]={
   
   0,  4,  8,  12, 16, 20, 24, 27, 31, 35, 39, 43, 47, 51, 55, 59,
   63, 67, 71, 75, 78, 82, 86, 90, 94, 98, 102,106,110,114,118,122,
   125,129,133,137,141,145,149,153,157,161,165,169,173,176,180,184,
   188,192,196,200,204,208,212,216,220,224,227,231,235,239,243,247,
   251,255,259,263,267,271,275,278,282,286,290,294,298,302,306,310,
   314,318,322,325,329,333,337,341,345,349,353,357,361,365,369,373,
   376,380,384,388,392,396,400,404,408,412,416,420,424,427,431,435,
   439,443,447,451,455,459,463,467,471,475,478,482,486,490,494,498,
   502,506,510,514,518,522,525,529,533,537,541,545,549,553,557,561,
   565,569,573,576,580,584,588,592,596,600,604,608,612,616,620,624,
   627,631,635,639,643,647,651,655,659,663,667,671,675,678,682,686,
   690,694,698,702,706,710,714,718,722,725,729,733,737,741,745,749,
   753,757,761,765,769,773,776,780,784,788,792,796,800,804,808,812,
   816,820,824,827,831,835,839,843,847,851,855,859,863,867,871,875,
   878,882,886,890,894,898,902,906,910,914,918,922,925,929,933,937,
   941,945,949,953,957,961,965,969,973,976,980,984,988,992,996,1000

};

#else

const int CCP_Values[256]={
   
    0,  2,  4,  6,  8,  10, 12, 14, 16, 18, 20, 21, 23, 25, 27, 29,
    31, 33, 35, 37, 39, 41, 43, 45, 47, 49, 51, 53, 55, 57, 59, 61,
    63, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92,
    94, 96, 98, 100,102,104,105,107,109,111,113,115,117,119,121,123,
    125,127,129,131,133,135,137,139,141,143,145,146,148,150,152,154,
    156,158,160,162,164,166,168,170,172,174,176,178,180,182,184,186,
    188,189,191,193,195,197,199,201,203,205,207,209,211,213,215,217,
    219,221,223,225,227,229,230,232,234,236,238,240,242,244,246,248,
    250,252,254,256,258,260,262,264,266,268,270,271,273,275,277,279,
    281,283,285,287,289,291,293,295,297,299,301,303,305,307,309,311,
    313,314,316,318,320,322,324,326,328,330,332,334,336,338,340,342,
    344,346,348,350,352,354,355,357,359,361,363,365,367,369,371,373,
    375,377,379,381,383,385,387,389,391,393,395,396,398,400,402,404,
    406,408,410,412,414,416,418,420,422,424,426,428,430,432,434,436,
    438,439,441,443,445,447,449,451,453,455,457,459,461,463,465,467,
    469,471,473,475,477,479,480,482,484,486,488,490,492,494,496,500
};
#endif
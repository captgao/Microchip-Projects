/*
 * File:   int.c
 * Author: zhgao
 *
 * Created on 2018?8?12?, ??8:47
 */


#include <xc.h>
#include "motor.h"
#define THRH_VOLT 0x300
void overload_protect(void);
void __interrupt() ISR(void){
    //LATC4 = 1;
    overload_protect();
    if(TMR2IF){
        TMR2IF = 0;
        motor_serv();
    }
   // LATC4 = 0;
}
void overload_protect(void){
    ADGO = 1;
    while(ADGO);
    unsigned int volt = ADRESH<<8||ADRESL;
    if(volt>THRH_VOLT){
        close_motor();
    }
}
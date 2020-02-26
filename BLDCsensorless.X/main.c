/*
 * File:   main.c
 * Author: zhgao
 *
 * Created on 2018?8?12?, ??7:50
 */
// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection->INTOSC oscillator: I/O function on CLKIN pin
#pragma config WDTE = OFF    // Watchdog Timer Enable->WDT disabled
#pragma config PWRTE = OFF    // Power-up Timer Enable->PWRT disabled
#pragma config MCLRE = OFF    // MCLR Pin Function Select->MCLR/VPP pin function is MCLR
#pragma config CP = OFF    // Flash Program Memory Code Protection->Program memory code protection is disabled
#pragma config CPD = OFF    // Data Memory Code Protection->Data memory code protection is disabled
#pragma config BOREN = ON    // Brown-out Reset Enable->Brown-out Reset enabled
#pragma config CLKOUTEN = OFF    // Clock Out Enable->CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin
#pragma config IESO = ON    // Internal/External Switchover->Internal/External Switchover mode is enabled
#pragma config FCMEN = ON    // Fail-Safe Clock Monitor Enable->Fail-Safe Clock Monitor is enabled

// CONFIG2
#pragma config WRT = OFF    // Flash Memory Self-Write Protection->Write protection off
#pragma config VCAPEN = OFF    // Voltage Regulator Capacitor Enable->All VCAP pin functionality is disabled
#pragma config PLLEN = ON    // PLL Enable->4x PLL enabled
#pragma config STVREN = ON    // Stack Overflow/Underflow Reset Enable->Stack Overflow or Underflow will cause a Reset
#pragma config BORV = LO    // Brown-out Reset Voltage Selection->Brown-out Reset Voltage (Vbor), low trip point selected.
#pragma config LVP = ON    // Low-Voltage Programming Enable->Low-voltage programming enabled

#include <xc.h>
#include "ain.h"
#include "motor.h"
#include <pic16f1936.h>
inline void TMR2_init(void);
inline void PWM_init(void);
inline void PORT_init(void);
inline void CMP1_init(void);
inline void ADC_init(void);
void OSCILLATOR_init(void);
void main(void) {
    OSCILLATOR_init();
    GIE = 1;
    PEIE = 1;
    PORT_init();
    PWM_init();
    CMP1_init();
    ADC_init();
    TMR2_init();
    start_motor();
    while(1){
    }
}
inline void ADC_init(void){
    FVRCON = 0b11000011;
    ADCON1 = 0b11010011;
    ADCON0 = 0b00010001;
    
}
inline void PORT_init(void){
    AIN1_SEL = 1;
    AIN3_SEL = 1;
    AIN4_SEL = 1;
    AIN5_SEL = 1;
    AIN7_SEL = 1;
    LATA = 0;
    LATB = 0;
    LATC = 0;
    LATC4 = 1;
    TRISA = 0xFF;
    TRISB = 0x8;
    TRISC = 0;
    
}
inline void TMR2_init(void)
{
    // Set TMR2 to the options selected in the User Interface
    // T2CKPS 1:4; T2OUTPS 1:1; TMR2ON off; 
    T2CON = 0x10;
    // T2PR 125; 
    PR2 = 0x20;
    // TMR2 0; 
    TMR2 = 0x00;
    // Clearing IF flag before enabling the interrupt.
    TMR2IF = 0;
    // Enabling TMR2 interrupt.
    TMR2IE = 1;
    TMR2ON = 1;
}
inline void PWM_init(void)
{
    TRISC2 = 1;
    TRISB2 = 1;
    TRISB1 = 1;
    CCP1CON = 0x0c;
    CCPR1L = 0x20;
    CCPTMRS0 = 0x01;
    PSTR1CON = 0;
    //TMR4 init
    T4CON = 0x0;
    PR4 = 0xFF;
    TMR4 = 0x0;
    TMR4IF = 0;
    TMR4IE = 0;
    TMR4ON = 1;
    while(TMR4IF == 0);
    TRISC2 = 0;
    TRISB2 = 0;
    TRISB1 = 0;
}
inline void CMP1_init(void)
{
    CM1CON0 = 0b10000100;
    CM1CON1 = 0;
}
void OSCILLATOR_init(void)
{
    // SCS INTOSC; SPLLEN disabled; IRCF 16MHz_HF; 
    OSCCON = 0x7A;
    // TUN 0; 
    OSCTUNE = 0x00;
    // SBOREN disabled; 
    BORCON = 0x00;
}
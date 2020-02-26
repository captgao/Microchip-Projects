/*
 * File:   main.c
 * Author: zhgao
 *
 * Created on 2018?7?25?, ??10:57
 */


#include <xc.h>
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (Internal HFINTOSC. I/O function on RA4 and RA5.)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit cannot be cleared once it is set by software)
#pragma config ZCDDIS = ON      // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR)
#pragma config PLLEN = ON       // Phase Lock Loop enable (4x PLL is always enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

int waitOK = 0;
int timebase;
int times;
void wait(int ms);
void main(void) {
    GIE = 1;
    LATA = 0xFF;
    LATC = 0xFF;
    TRISC = 0;
    TRISA = 0;
    wait(500);
    while(1){
    LATA0 = 0;
    wait(250);
    LATA1 = 0;
    wait(250);
    LATC1 = 0;
    wait(250);
    LATA4 = 0;
    wait(250);
    LATC2 = 0;
    wait(250);
    LATC4 = 0;
    wait(250);
    LATC0 = 0;
    wait(250);
    LATA5 = 0;
    wait(250);

    LATA = 0xFF;
    LATC = 0xFF;
    wait(500);
    }
    while(1);
}

void wait(int ms){
    waitOK = 0;
    T0CS = 0;
    PSA = 1;
    T0IF = 0;
    T0IE = 1;
    timebase = 0xAA;
    times = ms;
    TMR0 = timebase;
    while(!waitOK);
}
void __interrupt() isr(void)
{
    static int n = 0;
    if(T0IF){
        if(n<times){
            n++;
            T0IF = 0;
            T0IE = 1;
            TMR0 = timebase;
        }
        else{
            n = 0;
            waitOK = 1;
        }
    }
}
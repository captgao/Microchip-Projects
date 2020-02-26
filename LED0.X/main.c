
#include <xc.h>


// PIC16F1703 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
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

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
int waitOK = 0;
int timebase;
int times;
#define ADCMAX 0x3F
#define VTHR 137
void wdt_init(void);
unsigned int getadc();
void wait(int ms);
unsigned int vadc;
void main(void) {
    wdt_init();
    GIE = 1;
    //ADC INIT
    ADON = 1;
    ADFM = 1;
    ADCON1bits.ADCS = 0b110;
    ADCON0bits.CHS = 0x2;
    ADPREF0 = 0;
    ADPREF1 = 0;
    //PORT INIT
    TRISA = 0x0;
    TRISC = 0x0;
    TRISA2 = 1;
    TRISC3 = 1;
    LATA = 0x0;
    LATC = 0x0;
    ANSA2 =1;
    ANSC3 = 0;
    while(1){
       // LATC0 =0;
        vadc = getadc();
   //     voltage = VREF * vadc/ ADCMAX;
        if(vadc <= VTHR){
      //      LATC0 = 1;
            if(RC3){
                LATA0 = 1;
                LATA1 = 1;
                wait(250);
                LATC1 = 1;
                LATC0 = 1;
                wait(250);
                LATC2 = 1;
                LATC4 = 1;
                wait(250);
                LATA4 = 1;
                LATA5 = 1;
                wait(250);
                LATA5 = 0;
                LATA4 = 0;
                wait(250);
                LATC2 = 0;
                LATC4 = 0;
                wait(250);
                LATC1 = 0;
                LATC0 = 0;
                wait(250);
                LATA0 = 0;
                LATA1 = 0;
                wait(2000);
            }
            else{
                LATA0 = 1;
                LATA1 = 1;
                wait(250);
                LATA0 = 0;
                LATA1 = 0;
                LATC1 = 1;
                LATC0 = 1;
                wait(250);
                LATC1 = 0;
                LATC0 = 0;
                LATC2 = 1;
                LATC4 = 1;
                wait(250);
                LATC2 = 0;
                LATC4 = 0;
                LATA4 = 1;
                LATA5 = 1;
                wait(250);
                LATA5 = 0;
                LATA4 = 0;
                wait(2000);
            }
        }
        else{
            wait(2000);
        }
    }
}
void wdt_init(void){
    
} 
unsigned int getadc(){
    ADGO = 1;
    while(ADGO);
    return (unsigned int)((ADRESH<<8)|ADRESL );
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




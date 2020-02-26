#include <xc.h>
extern int timebase;
extern int times;
extern int waitOK;
void __interrupt() isr(void)
{
    if(ADIF){
        ADIF = 0;
    }
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

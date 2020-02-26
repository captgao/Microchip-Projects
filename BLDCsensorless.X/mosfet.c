#include <xc.h>
#include <pic16f1936.h>
#include "mosfet.h"
#include "motor.h"

#define PWM_OFF_UL STR1A=0
#define PWM_ON_UL  STR1A=1
#define PWM_OFF_VL STR1B=0
#define PWM_ON_VL STR1B=1
#define PWM_OFF_WL STR1C=0
#define PWM_ON_WL STR1C=1
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

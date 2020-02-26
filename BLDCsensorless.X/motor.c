/*
 * File:   motor.c
 * Author: zhgao
 *
 * Created on 2018?8?20?, ??4:08
 */


#include <xc.h>
#include "motor.h"
#include "mosfet.h"

/*typedef enum 
{
    COMM_OFF, 
    COMM_STEP1, COMM_STEP2, 
    COMM_STEP3, COMM_STEP4, 
    COMM_STEP5, COMM_STEP6, 
    COMM_STEPOVER
} CommuState;*/
CommuState commustate = COMM_OFF;
static unsigned int time_count = 0;
static unsigned char bemf_filter = 0;
static char zerocross = 0;
static char flag_start = 0;
static unsigned long phase_delay_filter = 0;
static unsigned int phase_delay = 0;
static unsigned long phase_delay_counter = 0;
static const GateState_t gateStates[] = {
    {UHoff, ULoff, VHoff, VLoff, WHoff, WLoff},
    {ULoff, VHoff, WHoff, WLoff, UHon, VLon},
    {ULoff, VHoff, VLoff, WHoff, UHon, WLon},
    {UHoff, ULoff, VLoff, WHoff, VHon, WLon},
    {UHoff, VLoff, WHoff, WLoff, VHon, ULon},
    {UHoff, VHoff, VLoff, WLoff, WHon, ULon},
    {UHoff, ULoff, VHoff, WLoff, WHon, VLon},
    {UHoff, ULoff, VHoff, VLoff, WHoff, WLoff},
};
static const unsigned char cBEMF_FILTER[64] = {0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10, 0x12, 0x14, 0x16, 0x18, 0x1A, 0x1C, 0x1E,
    0x20, 0x22, 0x24, 0x26, 0x28, 0x2A, 0x2C, 0x2E, 0x01, 0x01, 0x01, 0x36, 0x01, 0x3A, 0x3C, 0x3E,
    0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x01, 0x01, 0x01, 0x16, 0x01, 0x1A, 0x1C, 0x1E,
    0x01, 0x01, 0x01, 0x26, 0x01, 0x2A, 0x2C, 0x2E, 0x01, 0x01, 0x01, 0x36, 0x01, 0x3A, 0x3C, 0x01};
#define SLOWSTART_TIME 0x6E0
void motor_serv(void) {
    //static unsigned int force_count = MAX_Commtime;
    if (commustate == COMM_OFF) {
        if (flag_start == 1) {
            commustate = COMM_STEP1;
            state_drive(COMM_STEP1);
            set_cmp(commustate);
        }
        return;
    }
    time_count++;
    if (bemf_zerocross()) bemf_filter |= 1;
    bemf_filter = cBEMF_FILTER[bemf_filter];
    if (bemf_filter & 1) zerocross = 1;
   
    if (zerocross) {
        if (!(phase_delay_counter--)) {
            LATC4 = ~LATC4;
            //force_count = force_count - (force_count >> 4) + (time_count >> 4);
            commutate();
            return;
        }
        
    }
    
    if (time_count > MAX_Commtime) {
        commutate();
        return;
    }/*
    if(time_count > force_count + 2){
        commutate();
        return;
    }*/
}

void start_motor(void) {
    flag_start = 1;
}

unsigned char bemf_zerocross(void) {
    return (unsigned char) C1OUT;
}

void commutate(void) {
   // LATC4 = ~LATC4;
    PhaseDelayFilter();
    zerocross = 0;
    time_count = 0;
    bemf_filter = 0;
    if (commustate == COMM_STEP6) commustate = COMM_STEP1;
    else commustate++;
    state_drive(commustate);
    set_cmp(commustate);
}

void state_drive(CommuState state) {
    gateStates[state].drive[0]();
    gateStates[state].drive[1]();
    gateStates[state].drive[2]();
    gateStates[state].drive[3]();
    gateStates[state].drive[4]();
    gateStates[state].drive[5]();
}

void set_cmp(CommuState state) {
    switch (state) {
        case(COMM_STEP1):
        {
            //  W high to low
            C1POL = 0;
            C1NCH1 = 1;
            C1NCH0 = 0;
            break;
        }
        case(COMM_STEP2):
        {
            // V low to high
            C1POL = 1;
            C1NCH1 = 0;
            C1NCH0 = 1;
            break;
        }
        case(COMM_STEP3):
        {
            //U high to low
            C1POL = 0;
            C1NCH1 = 0;
            C1NCH0 = 0;
            break;
        }
        case(COMM_STEP4):
        {
            // W low to high
            C1POL = 1;
            C1NCH1 = 1;
            C1NCH0 = 0;
            break;
        }
        case(COMM_STEP5):
        {
            //V high to low
            C1POL = 0;
            C1NCH1 = 0;
            C1NCH0 = 1;
            break;
        }
        case(COMM_STEP6):
        {
            //U low to high
            C1POL = 1;
            C1NCH1 = 0;
            C1NCH0 = 0;
            break;
        }
        default:break;
    }
}

void close_motor(void) {
    state_drive(COMM_OFF);
    commustate = COMM_OFF;
}


static void PhaseDelayFilter(void) {
    phase_delay_filter += time_count;
    phase_delay = phase_delay_filter >> FILTER_DELAY;
    phase_delay_filter -= phase_delay;
    phase_delay_counter = phase_delay >> 3;
}
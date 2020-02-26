/*
    MICROCHIP SOFTWARE NOTICE AND DISCLAIMER:

    You may use this software, and any derivatives created by any person or
    entity by or on your behalf, exclusively with Microchip's products.
    Microchip and its subsidiaries ("Microchip"), and its licensors, retain all
    ownership and intellectual property rights in the accompanying software and
    in all derivatives hereto.

    This software and any accompanying information is for suggestion only. It
    does not modify Microchip's standard warranty for its products.  You agree
    that you are solely responsible for testing the software and determining
    its suitability.  Microchip has no obligation to modify, test, certify, or
    support the software.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE APPLY TO THIS SOFTWARE, ITS INTERACTION WITH MICROCHIP'S
    PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT, WILL MICROCHIP BE LIABLE, WHETHER IN CONTRACT, WARRANTY, TORT
    (INCLUDING NEGLIGENCE OR BREACH OF STATUTORY DUTY), STRICT LIABILITY,
    INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF
    ANY KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWSOEVER CAUSED, EVEN IF
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE
    FORESEEABLE.  TO THE FULLEST EXTENT ALLOWABLE BY LAW, MICROCHIP'S TOTAL
    LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED
    THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR
    THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF
    THESE TERMS.
*/
#include <xc.h>
#include <stdint.h>
#include <stdlib.h>

#include "../tmr2.h"

#include "mtouch_sensor.h"
#include "mtouch_sensor_scan.h"


#define PIC_ADC_RESOLUTION                                  ((mtouch_sensor_adcsample_t)1024)
#define Sensor_calculate_active_thrs(oversampling)          (mtouch_sensor_packetsample_t)(oversampling)<<2

typedef uint16_t mtouch_sensor_adcsample_t;
#define MTOUCH_SENSOR_ADCSAMPLE_MIN ((mtouch_sensor_adcsample_t)0)
#define MTOUCH_SENSOR_ADCSAMPLE_MAX ((mtouch_sensor_adcsample_t)1023)

typedef uint8_t mtouch_sensor_packetcounter_t;
#define MTOUCH_SENSOR_PACKET_ADCSAMPLES ((mtouch_sensor_packetcounter_t)32)
#define MTOUCH_SENSOR_PACKETCOUNTER_MIN ((mtouch_sensor_packetcounter_t)0)
#define MTOUCH_SENSOR_PACKETCOUNTER_MAX ((mtouch_sensor_packetcounter_t)UINT8_MAX)

typedef uint16_t mtouch_sensor_packetsample_t;
#define MTOUCH_SENSOR_PACKETSAMPLE_MIN ((mtouch_sensor_packetsample_t)0)
#define MTOUCH_SENSOR_PACKETSAMPLE_MAX ((mtouch_sensor_packetsample_t)UINT16_MAX)

typedef uint16_t mtouch_sensor_packetnoise_t;
#define MTOUCH_SENSOR_PACKETNOISE_MIN ((mtouch_sensor_packetnoise_t)0)
#define MTOUCH_SENSOR_PACKETNOISE_MAX ((mtouch_sensor_packetnoise_t)UINT16_MAX)

typedef uint8_t mtouch_sensor_sampleperiod_t;
#define MTOUCH_SENSOR_SAMPLEPERIOD_MIN  ((mtouch_sensor_sampleperiod_t)200)
#define MTOUCH_SENSOR_SAMPLEPERIOD_MAX  ((mtouch_sensor_sampleperiod_t)255) 

#define SCAN_RETRY                                                      (uint8_t)5

typedef void(*scanFunction)(void);

typedef struct
{
    unsigned    packet_done:1;
    unsigned    interrupted:1;
} mtouch_sensor_globalflags_t;

typedef struct
{
 const  enum mtouch_sensor_names        sensor_name;
 const  scanFunction                    scanA;
 const  scanFunction                    scanB;
        mtouch_sensor_packetcounter_t   oversampling;
        mtouch_sensor_sample_t          rawSample;
        unsigned                        enabled:1;
        unsigned                        sampled:1;
        unsigned                        acitve:1;
} mtouch_sensor_t;

/*
 * =======================================================================
 * LOCAL FUNCTIONS
 * =======================================================================
 */
static enum mtouch_sensor_error Sensor_Service              (enum mtouch_sensor_names name);
static void                     Sensor_setScanFunction      (mtouch_sensor_t* sensor);
static enum mtouch_sensor_error Sensor_Acquisition          (mtouch_sensor_t* sensor);
static enum mtouch_sensor_error Sensor_Acq_ExecutePacket    (mtouch_sensor_t* sensor);
static void                     Sensor_Acq_ExecuteScan      (void);

static inline void              Sensor_setSampled           (mtouch_sensor_t* sensor);
static inline void              Sensor_Sampled_Reset        (mtouch_sensor_t* sensor);
static inline void              Sensor_setActive            (mtouch_sensor_t* sensor);
static inline void              Sensor_setInactive          (mtouch_sensor_t* sensor);
static        bool              Sensor_isEnabled            (mtouch_sensor_t* sensor);

static void                     Sensor_RawSample_Update     (mtouch_sensor_t* sensor);

static void                     Sensor_DefaultCallback      (enum mtouch_sensor_names sensor);

static enum mtouch_sensor_error Sensor_Scanfrequency_Evaluation(mtouch_sensor_t* sensor);

/*
 * =======================================================================
 *  Callback Function Pointers
 * =======================================================================
 */
static void (*callback_sampled)(enum mtouch_sensor_names sensor) = Sensor_DefaultCallback;

/*
 * =======================================================================
 *  Local Variables
 * =======================================================================
 */
static volatile mtouch_sensor_globalflags_t   sensor_globalFlags;
static enum mtouch_sensor_names         currentScannSensor;
static mtouch_sensor_packetcounter_t    packet_counter;
static mtouch_sensor_packetsample_t     packet_sample;
static mtouch_sensor_sampleperiod_t     sample_period = MTOUCH_SENSOR_SAMPLEPERIOD_MIN;
static mtouch_sensor_packetnoise_t      packet_noise;
static scanFunction                     Sensor_scanA = MTOUCH_CVD_ScanA_0;
static scanFunction                     Sensor_scanB = MTOUCH_CVD_ScanB_0;
/*
 * =======================================================================
 *  Sensor Configurations
 * =======================================================================
 */
static mtouch_sensor_t mtouch_sensor[MTOUCH_SENSORS] ={
    {Sensor_AN5,MTOUCH_CVD_ScanA_0,MTOUCH_CVD_ScanB_0, 32,0,0,0,0},
};

/*
 * =======================================================================
 * MTOUCH_Sensor_Init()
 * =======================================================================
 */
enum mtouch_sensor_error MTOUCH_Sensor_Initialize(enum mtouch_sensor_names sensor)
{
    switch(sensor)                                  /* Overwrite TRIS,ANSEL and WPU*/
    {
        case 0:  
            WPUCbits.WPUC1 = 0;
            ANSELCbits.ANSC1 = 0;
            TRISCbits.TRISC1 = 0;
            break;
        default: return MTOUCH_SENSOR_ERROR_invalid_index;
    }
    
    MTOUCH_Sensor_Enable(sensor);
    Sensor_Sampled_Reset(&mtouch_sensor[sensor]);
    
    return MTOUCH_SENSOR_ERROR_none;
}


/*
 * =======================================================================
 * MTOUCH_Sensor_InitializeAll()
 * =======================================================================
 */
void MTOUCH_Sensor_InitializeAll(void)
{
    enum mtouch_sensor_names sensor; 
    for (sensor = 0; sensor < MTOUCH_SENSORS; sensor++)
    {
        MTOUCH_Sensor_Initialize(sensor);
    }
}

/*
 * =======================================================================
 * MTOUCH_SensorScan_Initialize
 * =======================================================================
 *  initialization for ADC and Timer module
 */
void MTOUCH_Sensor_Scan_Initialize(void)
{
    T2CONbits.T2CKPS = 0x0;

    ADCON0 = (uint8_t)0;                            /* overwrite the ADC configuration for mTouch scan */
    ADCON1 = (uint8_t)( 0x1<<7 | 0x2<<4 | 0x0 );
    ADCON2 = (uint8_t)0;
}


/*
 * =======================================================================
 * MTOUCH_Sensor_SampleAll()
 * =======================================================================
 *  
 */

bool MTOUCH_Sensor_SampleAll(void)
{
    enum mtouch_sensor_names sensor;   
    for (sensor = 0; sensor < MTOUCH_SENSORS; sensor++)
    {
        if(Sensor_Service(sensor)!= MTOUCH_SENSOR_ERROR_none)
            return false;
    }
    return true;
}


/*
 * =======================================================================
 * Sensor_Service()
 * =======================================================================
 */
static enum mtouch_sensor_error Sensor_Service(enum mtouch_sensor_names name)
{
    mtouch_sensor_t*       sensor = &mtouch_sensor[name];
    enum mtouch_sensor_error error = Sensor_Acquisition(sensor);

    /* Validate sensor output. Handle errors. */
    switch(error)
    {
        case MTOUCH_SENSOR_ERROR_none:
        {
            Sensor_RawSample_Update(sensor);
            Sensor_setSampled(sensor);
            callback_sampled(name);
        }
        break;
        default: break;
    }

    return error;
}


void MTOUCH_Sensor_NotifyInterruptOccurred(void)
{
    if(!sensor_globalFlags.packet_done)
        sensor_globalFlags.interrupted = true;
}
/*
 * =======================================================================
 * Sensor_Acquisition()
 * =======================================================================
 */
static enum mtouch_sensor_error Sensor_Acquisition(mtouch_sensor_t* sensor)
{

    uint8_t retry = SCAN_RETRY;
     
    if(!Sensor_isEnabled(sensor))
        return MTOUCH_SENSOR_ERROR_none;
    
    while(Sensor_Acq_ExecutePacket(sensor))
    {
        retry--;
        if(retry == 0)
        {
            return MTOUCH_SENSOR_ERROR_tooManyRetries;
        }
    }

    if((mtouch_sensor_packetsample_t)abs(packet_sample - sensor->rawSample) > Sensor_calculate_active_thrs(sensor->oversampling))
    {
        Sensor_setActive(sensor);
        //return Sensor_Scanfrequency_Evaluation(sensor);
    }
    else
    {
        Sensor_setInactive(sensor);
    }
    
    return MTOUCH_SENSOR_ERROR_none;
}


/*
 * =======================================================================
 * Sensor_Acq_ExecutePacket()
 * =======================================================================
 */
static enum mtouch_sensor_error Sensor_Acq_ExecutePacket(mtouch_sensor_t* sensor)
{
    /* software CVD with AFA requires interrupt enabled */
    if(!(INTCONbits.GIE & INTCONbits.PEIE))
        return MTOUCH_SENSOR_ERROR_interrupt_notEnabled;
        
    enum mtouch_sensor_error        error = MTOUCH_SENSOR_ERROR_none;
 /*   uint8_t ADCON0_temp;
    uint8_t ADCON1_temp;
    uint8_t ADCON2_temp;
    

    ADCON0_temp = ADCON0;       // store the current ADC configuration 
    ADCON1_temp = ADCON1;
    ADCON2_temp = ADCON2;*/
    MTOUCH_Sensor_Scan_Initialize();
    
                 
    Sensor_setScanFunction(sensor);  /* Setup the scan function */

    currentScannSensor = sensor->sensor_name;
    packet_counter  = sensor->oversampling;
    packet_sample = 0;
    sensor_globalFlags.packet_done = 0;
    packet_noise = 0;
    
    TMR2_SetInterruptHandler(Sensor_Acq_ExecuteScan);  /* Use timer2 to schedule the scan */
    TMR2_LoadPeriodRegister(sample_period);
    TMR2_StartTimer();
    
    sensor_globalFlags.interrupted = false;
    
    /* Perform packet samples */
    do
    {
        while(PIR1bits.ADIF == 0) 
        {
            if(sensor_globalFlags.packet_done == (uint8_t)1)
                break;
        }
        PIR1bits.ADIF = 0;    
    } while(sensor_globalFlags.packet_done == 0);


    TMR2_StopTimer();
    /* restore the previous ADC configuration */
   /* ADCON0 = ADCON0_temp;       
    ADCON1 = ADCON1_temp;
    ADCON2 = ADCON2_temp;*/
    
    if(sensor_globalFlags.interrupted)
    {
        error = MTOUCH_SENSOR_ERROR_interruptedScan;
    }
    
    return error;
}


/*
 * =======================================================================
 * Sensor_Acq_ExecuteScan()
 * =======================================================================
 * Perform a single sample on the sensor. This is a local function and
 * requires that the ExecutePacket() function guarantees the correct PIC
 * and scanning configuration.
 *
 * This function is written to be independent of mainloop vs ISR context.
 */
static void Sensor_Acq_ExecuteScan(void)
{
    mtouch_sensor_adcsample_t result = ADRES;       /* result from previous scan */
    static mtouch_sensor_adcsample_t last_a,last_b;

    if(sensor_globalFlags.packet_done)
        return;
    
    if (packet_counter != (uint8_t)0)
    {
        #pragma switch time
        switch(packet_counter & 0x01)
        {
            case 0: Sensor_scanA();break;
            case 1: Sensor_scanB();break;
            default: break;
        }
        /* Accumulate previous sample result during the ADC conversion */
        if(packet_counter!=mtouch_sensor[currentScannSensor].oversampling)
        {
            if(packet_counter & 0x01)
            {    
                result = PIC_ADC_RESOLUTION - result;
                packet_noise += (mtouch_sensor_packetsample_t)abs(last_a-result);
                last_a = result;
            }
            else
            {
                packet_noise += (mtouch_sensor_packetsample_t)abs(last_b-result);
                last_b = result;
            }
            packet_sample += result;
        }
        packet_counter--;
    }
    else
    {
        packet_sample += result;
        packet_noise += (mtouch_sensor_packetsample_t)abs(last_b-result);
        sensor_globalFlags.packet_done = (uint8_t)1;
    }
}

/*
 * 
 *=======================================================================
 * Automatic Frequency Adaptation
 *=======================================================================
 *
 */
static enum mtouch_sensor_error Sensor_Scanfrequency_Evaluation(mtouch_sensor_t* sensor)
{
    uint8_t i;
    const mtouch_sensor_sampleperiod_t  frequency_hop[5] = {3,13,14,20,13};
    mtouch_sensor_packetnoise_t         packet_noise_Max;
    mtouch_sensor_sampleperiod_t        best_sample_period;
    mtouch_sensor_packetsample_t        best_packet_sample;    
    uint8_t                             retry; 
    
    packet_noise_Max = packet_noise + (packet_noise>>2); /* put stickiness to the current scan frequency */
    best_sample_period = sample_period;
    best_packet_sample = packet_sample;
    

    for(i=(uint8_t)0;i<(uint8_t)5;i++)
    {   
        sample_period += frequency_hop[i];
        if(sample_period > MTOUCH_SENSOR_SAMPLEPERIOD_MAX)
        {
            sample_period-=MTOUCH_SENSOR_SAMPLEPERIOD_MAX;
            sample_period+=MTOUCH_SENSOR_SAMPLEPERIOD_MIN;
        }
        else if(sample_period < MTOUCH_SENSOR_SAMPLEPERIOD_MIN)
        {
            sample_period += MTOUCH_SENSOR_SAMPLEPERIOD_MIN;
        }
        
        retry = SCAN_RETRY;
        
        while(Sensor_Acq_ExecutePacket(sensor))
        {
            retry--;
            if(retry == 0)
            {
                return MTOUCH_SENSOR_ERROR_tooManyRetries;
            }
        }
        
        if(packet_noise_Max < packet_noise)
        {
            packet_noise_Max = packet_noise;
            best_sample_period = sample_period;
            best_packet_sample = packet_sample;
        }       
    }
    
    sample_period = best_sample_period;
    packet_sample = best_packet_sample;
    
    return MTOUCH_SENSOR_ERROR_none;
}

/*
 * =======================================================================
 * Sensor Raw Sample
 * =======================================================================
 */ 
mtouch_sensor_sample_t MTOUCH_Sensor_RawSample_Get(enum mtouch_sensor_names name) /* Global */
{
    if (name < MTOUCH_SENSORS)
    {
        return mtouch_sensor[name].rawSample;
    }
    else
        return (mtouch_sensor_sample_t)0;
}

static void Sensor_RawSample_Update(mtouch_sensor_t* sensor) /* Local */
{
    if (INTCONbits.GIE == (uint8_t)1)
    {
        INTCONbits.GIE = (uint8_t)0;
        sensor->rawSample = packet_sample;
        INTCONbits.GIE = (uint8_t)1;
    }
    else
    {
        sensor->rawSample = packet_sample;
    }
}


/*
 * =======================================================================
 * Sensor Sampled Callback
 * =======================================================================
 */ 
static void Sensor_DefaultCallback(enum mtouch_sensor_names sensor) { }
void MTOUCH_Sensor_SetSampledCallback(void (*callback)(enum mtouch_sensor_names sensor))
{
    callback_sampled = callback;
}

/*
 * =======================================================================
 * Scan Setter Function
 * =======================================================================
 */ 

static void Sensor_setScanFunction(mtouch_sensor_t* sensor)
{
    Sensor_scanA = sensor->scanA;
    Sensor_scanB = sensor->scanB;
}


/*
 * =======================================================================
 *  Enable/Disable Sensor
 * =======================================================================
 * 
 */
/*
void MTOUCH_Sensor_Disable(enum mtouch_sensor_names sensor)
{
    if(sensor < MTOUCH_SENSORS)   
        mtouch_sensor[sensor].enabled = 0;
}

void MTOUCH_Sensor_Enable(enum mtouch_sensor_names sensor)
{
    if(sensor < MTOUCH_SENSORS)   
        mtouch_sensor[sensor].enabled = 1;
}

bool MTOUCH_Sensor_isEnabled(enum mtouch_sensor_names sensor)
{
    if(sensor < MTOUCH_SENSORS)   
        return (bool)mtouch_sensor[sensor].enabled;
    else
        return false;
}

static bool Sensor_isEnabled(mtouch_sensor_t* sensor)
{
    if(sensor!=NULL)
        return (bool)sensor->enabled;
    else
        return false;
}
*/
/*
 * =======================================================================
 *  Sensor active status
 * =======================================================================
 * 
 */

static inline void Sensor_setActive(mtouch_sensor_t* sensor)
{
    sensor->acitve = 1;
}

static inline void Sensor_setInactive(mtouch_sensor_t* sensor)
{
    sensor->acitve = 0;
}
/*

bool MTOUCH_Sensor_isActive(enum mtouch_sensor_names sensor)
{
    if(sensor <= MTOUCH_SENSORS)
        return (bool)mtouch_sensor[sensor].acitve;
    return false;
}
*/

bool MTOUCH_Sensor_isCalibrated(enum mtouch_sensor_names sensor)
{
    /* always return true as software CVD implementation doesn't support analog calibration */
    return true;    
}

void MTOUCH_Sensor_Calibrate(enum mtouch_sensor_names sensor)
{
    /* Empty function as software CVD implementation doesn't support analog calibration*/
}

/*
 * =======================================================================
 *  Sensor sample status
 * =======================================================================
 * 
 */

void MTOUCH_Sensor_Sampled_ResetAll(void)
{
    mtouch_sensor_t* sensor;
    for(sensor = &mtouch_sensor[0];sensor<= &mtouch_sensor[MTOUCH_SENSORS-1];sensor++)
    {      
        Sensor_Sampled_Reset(sensor);
    }
}

bool MTOUCH_Sensor_wasSampled(enum mtouch_sensor_names sensor) 
{
   // if(sensor < MTOUCH_SENSORS)
        return (bool)mtouch_sensor[sensor].sampled;
   // else
     //   return false;
}

static inline void Sensor_Sampled_Reset(mtouch_sensor_t* sensor) 
{
    sensor->sampled = 0;
}

static inline void Sensor_setSampled(mtouch_sensor_t* sensor) 
{
    sensor->sampled = 1;
}

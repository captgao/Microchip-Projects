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
#ifndef MTOUCH_SENSOR_H
#define MTOUCH_SENSOR_H
  
    #include <stdint.h>
    #include <stdbool.h>
        
    #define MTOUCH_SENSORS      1
    
    
    enum mtouch_sensor_names
    {
        Sensor_AN5 = 0
    };
    
    enum mtouch_sensor_error
    {
        MTOUCH_SENSOR_ERROR_none            =  0,
        MTOUCH_SENSOR_ERROR_invalid_index   = -1,
        MTOUCH_SENSOR_ERROR_interrupt_notEnabled = -2,
        MTOUCH_SENSOR_ERROR_invalid_calibrate= -3,
        MTOUCH_SENSOR_ERROR_tooManyRetries = -4,
        MTOUCH_SENSOR_ERROR_scanOverrun = -5,
        MTOUCH_SENSOR_ERROR_interruptedScan = -6
    };

/*
 * =======================================================================
 * Typedefs / Data Types
 * =======================================================================
 */
    typedef uint16_t mtouch_sensor_sample_t;
    #define MTOUCH_SENSOR_SAMPLE_MIN (0)
    #define MTOUCH_SENSOR_SAMPLE_MAX (UINT16_MAX)

    typedef uint8_t mtouch_sensor_mask_t;
    #define MTOUCH_SENSOR_MASK_MIN (0)
    #define MTOUCH_SENSOR_MASK_MAX (UINT8_MAX)

/*
 * =======================================================================
 * Global Functions
 * =======================================================================
 */
    
    enum mtouch_sensor_error    MTOUCH_Sensor_Initialize        (enum mtouch_sensor_names sensor);
    void                        MTOUCH_Sensor_Scan_Initialize   (void);
    void                        MTOUCH_Sensor_InitializeAll     (void);
    void                        MTOUCH_Sensor_SetSampledCallback(void (*callback)(enum mtouch_sensor_names sensor));
    void                        MTOUCH_Sensor_SampleAll         (void);
    
    bool                        MTOUCH_Sensor_isActive          (enum mtouch_sensor_names sensor);
    bool                        MTOUCH_Sensor_wasSampled        (enum mtouch_sensor_names sensor);
    bool                        MTOUCH_Sensor_isCalibrated      (enum mtouch_sensor_names sensor);
    bool                        MTOUCH_Sensor_isEnabled         (enum mtouch_sensor_names sensor);
    void                        MTOUCH_Sensor_Sampled_ResetAll  (void);
    void                        MTOUCH_Sensor_Disable           (enum mtouch_sensor_names sensor);
    void                        MTOUCH_Sensor_Enable            (enum mtouch_sensor_names sensor);
    void                        MTOUCH_Sensor_Calibrate         (enum mtouch_sensor_names sensor); 
    mtouch_sensor_sample_t      MTOUCH_Sensor_RawSample_Get     (enum mtouch_sensor_names sensor);
    
    void                        MTOUCH_Sensor_NotifyInterruptOccurred(void);
    typedef void(*scanFunction)(void);
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


#endif
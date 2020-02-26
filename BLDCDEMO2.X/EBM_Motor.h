// File: EBM_Motor.h
//
// Motor parameters for the copper topped motor supplied with BLDC Add-on demo board.
//
///////////////////////////////////////////////////////////////////////////////
// Parameters that should be adjusted to match the motor being used:
//
// !!!!Note: Replace the name of any other motor header file with the name of !!!!!
// !!!!!!!!! this file in the BLDC.h header file !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//    
// NUM_POLES - This is the number of magnetic poles in the motor's permanent magnet
//             You can determine the number of poles by feeling the number of
//             detents in one mechanical revolution and dividing that number by 3.
//             NUM_POLES is always an even number and cannot be less than 2.
//
// START_RPM - This determines the open-loop commutation time at startup. A good
//             starting point is 1/10th the expected motor speed at the startup voltage.
//
// STARTUP_DUTY_CYCLE - This is the startup duty cycle which determines the startup
//             voltage. Voltage = AppliedVoltage * Duty_cycle_count/MAX_DUTY_CYCLE.
//             Example: Applied voltage = 80V, STARTUP_DUTY_CYCLE = 0x10, therefore:
//             Startup voltage = 80 * 16/125 = 10.24 volts.
//
// MAX_STARTUP_EVENTS - The startup algorithm starts by single stepping the motor
//             this number of times. This is done to position the motor in a known
//             alignment before higher speed commutation is attempted. At least two
//             steps are needed because at the first step the motor may be prepositioned 
//             in a state where it cannot swing left or right because it's in the middle 
//             of that boundary. The second step assures that the motor will respond.
//
// TIMEBASE_SLOW_STEP - The time to dwell at each single step commutation during pre-
//             start. Some motors have very high inertia to overcome. The slow step
//             time should be only large enough to allow the motor to reach and stabilize
//             at each slow start commutation.
//
// TIMEBASE_DUTY_RAMP - At startup the applied motor voltage is set to the minimum.
//            The applied voltage starts to ramp up after commutation lock is detected 
//            at the startup voltage. TIMEBASE_DUTY_RAMP determines the time to dwell
//            at each ramp-up step thereby slowing the rate at which the applied voltage
//            is increased. High inertia systems need more time than low inertia systems.
//
// BLANKING_COUNT_us - The number of microseconds to hold off from detecting zero cross
//            immediately after a commutation event. This allows the flyback currents
//            to die out so the back EMF can be accurately measured. Flyback current
//            causes the large spike in the back EMF voltage immediately after releasing
//            the motor phase from active drive current.
//
// STALL_COUNT_us - This is the shortest expected commutation time. When the motor stalls
//             a false zero cross is detected immediately after every commutation. This causes 
//             the zero cross tracking algorithm to repeatedly decrease the commutation time
//             at each commutation event until the commutation becomes shorter than the motor's 
//             top rated speed. The control loop recognizes this and reverts back to the 
//             startup procedure when it happens.
//
// RAMP_INCR - This is used only for main motor drive programs that include high-inertia 
//             startup. During the open-loop phase of startup, RAMP_INCR determines the next
//             commutation time as a percentage of the previous commutation time. Larger
//             system inertia requires smaller percentage.
//
// ERROR_SCALE - Feedback scaling factor.
//             The error is the difference between the expected zero cross time and the
//             actual zero cross time. The error is scaled down before it is accumulated into
//             into the commutation time. 2 raised to ERROR_SCALE (2^ERROR_SCALE) is the scaling
//             division factor. If the scaling factor is too large then the motor response will
//             be slow. If the scaling factor is too small then the motor may become unstable.
//             If the motor frequently misses lock after the startup sequence then this number is 
//             probably too small. If the motor loses lock at high speed or during acceleratioin
//             then this number is probably too large.
//
// HIGH_INERTIA - Define this variable for systems with slow response times.
//
///////////////////////////////////////////////////////////////////////////////

/************************************************************************
* Motor Definitions                                                               
*************************************************************************/
// Comment the following line out for low inertia systems
//#define HIGH_INERTIA

// # of permanent magnet poles
#define NUM_POLES              4L

// # of phases in the motor
#define NUM_PHASES             3L

// # of commutations in one mechanical revolution
#define COMM_PER_REV           (NUM_POLES*NUM_PHASES)

// start speed in RPM
#define START_RPM              300L

// PWM freq in hertz 
#define PWM_FREQ               16000L

// 60 seconds in one minute
#define SEC_PER_MIN            60L

// startup commutations per minute
#define START_COMM_PER_MIN    (START_RPM*COMM_PER_REV)

// commutations per second at beginning of startup ramp
#define START_COMM_PER_SEC    (START_COMM_PER_MIN/SEC_PER_MIN)

// Timer1 counts per commutation at beginning of startup
#define TMR1_START_COUNT      (TMR1_COUNTS_PER_SEC/START_COMM_PER_SEC) // 1000000/26

// initial starup commutation period
#define COMM_TIME_INIT			TMR1_START_COUNT
//#define COMM_TIME_INIT        0x0000

// Duty cycle for open loop operation
#define FIXED_DUTY_CYCLE      0x20

// Startup voltage is STARTUP_DUTY_CYCLE/0x7F * Motor Voltage                              
#define STARTUP_DUTY_CYCLE		0x20

// +1 forces PWM to 100% duty cycle
#define MAX_DUTY_CYCLE        ((PWM_PERIOD + 1) & 0xFF)

// xxx_STARTUP_DRIVE_PCT = determines initial CCPR1L duty cycle from speed table for motor startup
// NOTE: A CCPR1L number that seems to work best for most applications is 13%
//       Larger numbers tend to make the control algorithm jump past the 
//       ideal commutation rate during startup.
//       Smaller numbers usually do not produce enough torque to start the motor.
// There are three startup values. The one to use will be determined by measuring
// the motor supply voltage before startup.
#define LOW_STARTUP_DRIVE_PCT       15L
#define MED_STARTUP_DRIVE_PCT       13L
#define HI_STARTUP_DRIVE_PCT        11L

#define LOW_STARTUP_DUTYCYCLE       ((LOW_STARTUP_DRIVE_PCT*MAX_DUTY_CYCLE*4L)/100L)
#define MED_STARTUP_DUTYCYCLE       ((MED_STARTUP_DRIVE_PCT*MAX_DUTY_CYCLE*4L)/100L) //65
#define HI_STARTUP_DUTYCYCLE        ((HI_STARTUP_DRIVE_PCT*MAX_DUTY_CYCLE*4L)/100L)

// maximum sequential startup events before stop
#define MAX_STARTUP_EVENTS       2

// PWM duty cycle rate of change in response to speed control
// TIMEBASE_DUTY_RAMP*10ms = time between steps
#define TIMEBASE_DUTY_RAMP       1

// dwell time at slow step commutation: TIMEBASE_SLOW_STEP*10ms = dwell time
#define TIMEBASE_SLOW_STEP       10
                                          
// number of slow commutations between warmup and startup                                          
#define SLOW_STEPS               3

// startup ramp - specifies ramp step as percentage of comm time in number of right shifts. 
// Example: >>6 = 1/64 = .0156 = 1.56% step. This minimizes the 1/X effect of a fixed step.                                         
#define RAMP_INCR                6

// blanking count in microseconds
#define BLANKING_COUNT_us		   100L 

// stall commutation time in microseconds
#define  STALL_COUNT_us          800L             

// # of Timer1 counts below which a stall condition is detected
#define MICROSECONDS_PER_SECOND  1000000L
#define MIN_COMM_TIME            (STALL_COUNT_us * TMR1_COUNTS_PER_SEC)/MICROSECONDS_PER_SECOND

// The raw error is divided by 2 to the power of ERROR_SCALE before accumulating.
// Example: If the raw error is 96 and ERROR_SCALE is 3 then the error correction that is accumulated
// is reduced to 96/2^3 or 12.
#define ERROR_SCALE           3

// Number of microseconds to commutate early after zero cross
// this number is subtracted from half the expected commutation time to set commutation event after zero cross
#define ADVANCE_TIMING_us         0L
// Commutation happens in two stages
// Stage 1 is zero cross detection: Commutation is forced 1/2 commutation period after the Z-C event.
// Stage 2 is fixed commutation: Commutation timer is set at beginning for full commutation period.
// Stage 1 takes 12 us to detect Z-C and restuff the commutation timer. FIXED_ADVANCE_us adjusts for that difference.
#define FIXED_ADVANCE_TIMING_us   ADVANCE_TIMING_us - 0L

#define ADVANCE_COUNT            (ADVANCE_TIMING_us*TMR1_COUNTS_PER_us)
#define FIXED_ADVANCE_COUNT      (FIXED_ADVANCE_TIMING_us*TMR1_COUNTS_PER_us)

//////////////////////////////////////////////////////////////////////////////////////////
/////////////////////// Closed loop speed control parameters /////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

// Speed control duty cycle register range
#define DELTA_CONTROL    (MAX_DUTY_CYCLE-STARTUP_DUTY_CYCLE)

// Speed control duty cycle register at slowest speed
#define LO_DUTY_CYCLE        0x20

// Commutation period at fastest speed
#define HI_COMM_PERIOD_us    1572L

// Commutation period at slowest speed
#define LO_COMM_PERIOD_us   7667L
#define DELTA_COMM          (LO_COMM_PERIOD_us-HI_COMM_PERIOD_us)

// ADC reading at which the motor will run at the fastest speed
// in this case that is 95% of full scale
#define HI_ADC               250L

// ADC reading at which the motor will run at the slowest speed
// In this case that is 20% of full scale
#define LO_ADC                30L
#define DELTA_ADC             (HI_ADC-LO_ADC)

// Scaling factor used to convert ADC reading to desired commutation time
// The factor is multiplied by UP_SCALE to conserve fractional part while doing integer math.
// It helps if UP_SCALE is an even power of 2. It helps even more if that power is 8.
#define UP_SCALE             256L
#define SPEED_SCALE_FACTOR  (DELTA_COMM*UP_SCALE)/DELTA_ADC
#define ERROR_FACTOR        (DELTA_COMM*UP_SCALE)/DELTA_CONTROL
#define SPEED_RESOLUTION     SPEED_SCALE_FACTOR/UP_SCALE
#define CONTROL_RESOLUTION   DELTA_COMM/DELTA_CONTROL

// Commutation period averaging
// The averaging factor is log2(N) where N is the number of summations in the average
#define AVERAGING_FACTOR     8   

//////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// Low speed On-Off limits ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

// maximum number that will come from ADC speed request
#define MAX_SPEED_REQUEST        255

////////////////////////////////////////////////////////////////////////////////////////
//// On-Off limits

// percentage of speed request below which the motor will turn off
#define LOW_OFF_REQUEST_PCT      15

// percentage of speed request above which the motor will turn on
#define LOW_RESTORE_REQUEST_PCT  25

#define REQUEST_OFF             ((MAX_SPEED_REQUEST*LOW_OFF_REQUEST_PCT)/100)
#define REQUEST_ON              ((MAX_SPEED_REQUEST*LOW_RESTORE_REQUEST_PCT)/100)

//////////////////////////////////////////////////////////////////////////////////////////
////////////////////////// Speed control averaging factors ///////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

// ADC averaging factor
// Number of samples in the ADC average = 2^ADC_AVG_FACTOR
#define ADC_AVG_FACTOR          2

#pragma once
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#include <Arduino.h>
#include "macros.h"
#include "tmc2130.h"
#include "assembly_math.h"
#include "speed_lookuptable.h"

//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------

#ifdef HAS_TMC2130
#define MAXIMUM_STEPPER_RATE            5000000UL  // TMC* max rate
//#  define STEPPER_DIR_HIGH LOW
//#  define STEPPER_DIR_LOW  HIGH
#else
// A4988
#define MAXIMUM_STEPPER_RATE            500000UL  // a4988 max rate
#  define STEPPER_DIR_HIGH HIGH
#  define STEPPER_DIR_LOW  LOW
#endif

//------------------------------------------------------------------------------
// timing stuff
//------------------------------------------------------------------------------

#ifdef CPU_32_BIT
#  define ISR_BASE_CYCLES                 792UL
#  define ISR_LOOP_BASE_CYCLES            4UL
#  define ISR_STEPPER_CYCLES              16UL
// S curve interpolation adds 40 cycles
#  if ENABLED(S_CURVE_ACCELERATION)
#    define ISR_S_CURVE_CYCLES 40UL
#  else
#    define ISR_S_CURVE_CYCLES 0UL
#  endif
#else
#  define ISR_BASE_CYCLES                 752UL
#  define ISR_LOOP_BASE_CYCLES            32UL
#  define ISR_STEPPER_CYCLES              88UL

// S curve interpolation adds 160 cycles0
#  if ENABLED(S_CURVE_ACCELERATION)
#    define ISR_S_CURVE_CYCLES 160UL
#  else
#    define ISR_S_CURVE_CYCLES 0UL
#  endif
#endif


#define MIN_ISR_LOOP_CYCLES             (ISR_STEPPER_CYCLES * NUM_MUSCLES)
#define MINIMUM_STEPPER_PULSE           1UL

// TODO a guess.  use real math here!
// https://reprap.org/wiki/Step_rates
#define CLOCK_MIN_STEP_FREQUENCY (F_CPU / MAXIMUM_STEPPER_RATE)
  
#define _MIN_STEPPER_PULSE_CYCLES(N) _MAX( uint32_t((F_CPU) / (MAXIMUM_STEPPER_RATE)),  ((F_CPU) / 500000UL) * (N) )
  
#define MIN_STEPPER_PULSE_CYCLES       _MIN_STEPPER_PULSE_CYCLES(uint32_t(MINIMUM_STEPPER_PULSE))
#define ISR_LOOP_CYCLES                (ISR_LOOP_BASE_CYCLES + _MAX(MIN_STEPPER_PULSE_CYCLES, MIN_ISR_LOOP_CYCLES))
  
#define ISR_EXECUTION_CYCLES(R)  (  ( ISR_BASE_CYCLES + ISR_S_CURVE_CYCLES + (ISR_LOOP_CYCLES * (R)) ) / (R) )

// The maximum allowable stepping frequency when doing x128-x1 stepping (in Hz)
#define MAX_STEP_ISR_FREQUENCY_128X ((F_CPU) / ISR_EXECUTION_CYCLES(128))
#define MAX_STEP_ISR_FREQUENCY_64X  ((F_CPU) / ISR_EXECUTION_CYCLES(64))
#define MAX_STEP_ISR_FREQUENCY_32X  ((F_CPU) / ISR_EXECUTION_CYCLES(32))
#define MAX_STEP_ISR_FREQUENCY_16X  ((F_CPU) / ISR_EXECUTION_CYCLES(16))
#define MAX_STEP_ISR_FREQUENCY_8X   ((F_CPU) / ISR_EXECUTION_CYCLES(8))
#define MAX_STEP_ISR_FREQUENCY_4X   ((F_CPU) / ISR_EXECUTION_CYCLES(4))
#define MAX_STEP_ISR_FREQUENCY_2X   ((F_CPU) / ISR_EXECUTION_CYCLES(2))
#define MAX_STEP_ISR_FREQUENCY_1X   ((F_CPU) / ISR_EXECUTION_CYCLES(1))

//------------------------------------------------------------------------------
// enable pin.  reverse if your board or drivers are odd.
//------------------------------------------------------------------------------

#ifndef MOTOR_ENABLE_ON
#define MOTOR_ENABLE_ON  LOW
#endif

#ifndef MOTOR_ENABLE_OFF
#define MOTOR_ENABLE_OFF  HIGH
#endif

//------------------------------------------------------------------------------
// step direction
//------------------------------------------------------------------------------

#ifndef START0
#  define START0 LOW
#endif
#ifndef START1
#  define START1 LOW
#endif
#ifndef START2
#  define START2 LOW
#endif
#ifndef START3
#  define START3 LOW
#endif
#ifndef START4
#  define START4 LOW
#endif
#ifndef START5
#  define START5 LOW
#endif

#ifndef END0
#  define END0 HIGH
#endif
#ifndef END1
#  define END1 HIGH
#endif
#ifndef END2
#  define END2 HIGH
#endif
#ifndef END3
#  define END3 HIGH
#endif
#ifndef END4
#  define END4 HIGH
#endif
#ifndef END5
#  define END5 HIGH
#endif

//------------------------------------------------------------------------------
// loop unrolling
//------------------------------------------------------------------------------


#if NUM_MOTORS == 0
#define ALL_MOTOR_MACRO(NN) 
#endif
#if NUM_MOTORS == 1
#define ALL_MOTOR_MACRO(NN) NN(0) 
#endif
#if NUM_MOTORS == 2
#define ALL_MOTOR_MACRO(NN) NN(0) NN(1)
#endif
#if NUM_MOTORS == 3
#define ALL_MOTOR_MACRO(NN) NN(0) NN(1) NN(2)
#endif
#if NUM_MOTORS == 4
#define ALL_MOTOR_MACRO(NN) NN(0) NN(1) NN(2) NN(3)
#endif
#if NUM_MOTORS == 5
#define ALL_MOTOR_MACRO(NN) NN(0) NN(1) NN(2) NN(3) NN(4)
#endif
#if NUM_MOTORS == 6
#define ALL_MOTOR_MACRO(NN) NN(0) NN(1) NN(2) NN(3) NN(4) NN(5)
#endif


#if NUM_AXIES == 0
#define ALL_AXIS_MACRO(NN) 
#endif
#if NUM_AXIES == 1
#define ALL_AXIS_MACRO(NN) NN(0) 
#endif
#if NUM_AXIES == 2
#define ALL_AXIS_MACRO(NN) NN(0) NN(1)
#endif
#if NUM_AXIES == 3
#define ALL_AXIS_MACRO(NN) NN(0) NN(1) NN(2)
#endif
#if NUM_AXIES == 4
#define ALL_AXIS_MACRO(NN) NN(0) NN(1) NN(2) NN(3)
#endif
#if NUM_AXIES == 5
#define ALL_AXIS_MACRO(NN) NN(0) NN(1) NN(2) NN(3) NN(4)
#endif
#if NUM_AXIES == 6
#define ALL_AXIS_MACRO(NN) NN(0) NN(1) NN(2) NN(3) NN(4) NN(5)
#endif

//------------------------------------------------------------------------------

#define GET_AXIS_NAME(NN) (pgm_read_byte_near(AxisNames+NN))

//------------------------------------------------------------------------------
// STRUCTURES
//------------------------------------------------------------------------------

typedef struct {
  char letter;
  uint8_t step_pin;
  uint8_t dir_pin;
  uint8_t enable_pin;
  uint8_t limit_switch_pin;
} Motor;

class Stepper {
public:

#if defined(S_CURVE_ACCELERATION)
  static int32_t bezier_A,     // A coefficient in Bézier speed curve
                  bezier_B,     // B coefficient in Bézier speed curve
                  bezier_C;     // C coefficient in Bézier speed curve
  static uint32_t bezier_F,    // F coefficient in Bézier speed curve
                  bezier_AV;   // AV coefficient in Bézier speed curve
  #ifdef __AVR__
    static bool A_negative;    // If A coefficient was negative
  #endif
  static bool bezier_2nd_half; // If Bézier curve has been initialized or not
#else
  static uint32_t acc_step_rate; // needed for deceleration start point
#endif

  static uint32_t steps_total;
  static uint32_t steps_taken;
  static uint32_t accel_until;
  static uint32_t decel_after;
  static uint8_t isr_step_multiplier;
  static uint16_t directionBits;
  static uint32_t advance_divisor;
  
  static uint8_t oversampling_factor;

  static int32_t isr_nominal_rate;
  static uint32_t acceleration_time, deceleration_time;
  static uint32_t min_segment_time_us;

  static int32_t delta_error[NUM_MUSCLES];
  static int32_t over[NUM_MUSCLES];
  static int32_t count_position[NUM_MUSCLES];
  static int8_t count_direction[NUM_MUSCLES];

  static void setup();
  static void home();
  static void engage();
  static void disengage();
  static void set_step_count(int32_t *a);
  static void setPenAngle(int arg0);
  static void clockISRProfile();

  static void onestep(int motor);

  static bool isBlockBusy(const Segment *block);

  static void setDirections(uint16_t bits);

  static void isr();
  static void isrPulsePhase();
  static hal_timer_t isrBlockPhase();

#if defined(S_CURVE_ACCELERATION)
  static void _calc_bezier_curve_coeffs(const int32_t v0, const int32_t v1, const uint32_t av);
  static int32_t _eval_bezier_curve(const uint32_t curr_step);
#endif

  /**
     Set the clock 2 timer frequency.
    @input desired_freq_hz the desired frequency
  */
  FORCE_INLINE static uint32_t calc_interval(uint32_t step_rate, uint8_t * loops) {
    uint32_t timer;
    uint8_t step_multiplier = 1;
    
    step_rate <<= oversampling_factor;

    // The stepping frequency limits for each multistepping rate
    static const uint32_t limit[] PROGMEM = {
      (  MAX_STEP_ISR_FREQUENCY_1X     ),
      (  MAX_STEP_ISR_FREQUENCY_2X >> 1),
      (  MAX_STEP_ISR_FREQUENCY_4X >> 2),
      (  MAX_STEP_ISR_FREQUENCY_8X >> 3),
      ( MAX_STEP_ISR_FREQUENCY_16X >> 4),
      ( MAX_STEP_ISR_FREQUENCY_32X >> 5),
      ( MAX_STEP_ISR_FREQUENCY_64X >> 6),
      (MAX_STEP_ISR_FREQUENCY_128X >> 7)
    };

    int idx=0;
    while( idx<7 && step_rate > (uint32_t)pgm_read_dword(&limit[idx]) ) {
      step_rate >>= 1;
      step_multiplier <<= 1;
      ++idx;
    }
    *loops = step_multiplier;


    #ifdef CPU_32_BIT
      timer = uint32_t(STEPPER_TIMER_RATE) / desired_freq_hz;
    #else
      NOLESS(step_rate,CLOCK_MIN_STEP_FREQUENCY);
      step_rate -= CLOCK_MIN_STEP_FREQUENCY;
      if(step_rate >= 8 * 256) {
        const uint8_t tmp_step_rate  = (step_rate & 0x00FF);
        const uint16_t table_address = (uint16_t)&speed_lookuptable_fast[(uint8_t)(step_rate >> 8)][0],
                       gain          = (uint16_t)pgm_read_word_near(table_address + 2);
        timer                        = MultiU16X8toH16(tmp_step_rate, gain);
        timer                        = (uint16_t)pgm_read_word_near(table_address) - timer;
      } else {  // lower step rates
        uint16_t table_address = (uint16_t)&speed_lookuptable_slow[0][0];
        table_address += ((step_rate) >> 1) & 0xFFFC;
        timer = (uint16_t)pgm_read_word_near(table_address) -
                (((uint16_t)pgm_read_word_near(table_address + 2) * (uint8_t)(step_rate & 0x0007)) >> 3);
      }
    #endif

    return timer;
  }
};

#define MIN_STEP_ISR_FREQUENCY (MAX_STEP_ISR_FREQUENCY_1X/2)

//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------

extern Motor motors[NUM_MUSCLES];
extern const char AxisNames[] PROGMEM;

// max jerk value per axis
extern float max_jerk[NUM_MUSCLES];
// maximum steps/s per motor/servo 
extern float max_step_rate[NUM_MUSCLES];
// motor steps-per-unit.  one value per motor/servo
extern float motor_spu[NUM_MUSCLES];

extern Stepper motor;
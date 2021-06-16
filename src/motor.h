#pragma once
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#include <Arduino.h>

#include "macros.h"
#include "tmc2130.h"

#if NUM_SERVOS > 0
#ifdef USE_ALT_SERVO
#  include "mservo.h"
#else
#  include <Servo.h>
#endif
#endif

//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------

#ifdef HAS_TMC2130
#else
// A4988
#  define STEPPER_DIR_HIGH HIGH
#  define STEPPER_DIR_LOW  LOW
#endif

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
  static void setup();
  static void home();
  static void engage();
  static void disengage();
  static void set_step_count(long *a);
  static void setPenAngle(int arg0);
  static void clockISRProfile();

  static void onestep(int motor);

  static bool isBlockBusy(const Segment *block);

#ifdef DEBUG_STEPPING
  extern void debug_stepping();
#endif  // DEBUG_STEPPING
};

//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------

extern Motor motors[NUM_MUSCLES];
extern const char *AxisNames;

// max jerk value per axis
extern float max_jerk[NUM_MUSCLES];

// maximum feedrate (units/s).  one value per motor/servo 
extern float max_step_rate_s[NUM_MUSCLES];

// motor steps-per-unit.  one value per motor/servo
extern float motor_spu[NUM_MUSCLES];

extern uint32_t min_segment_time_us;

#if NUM_SERVOS>0
#ifndef ESP8266
extern Servo servos[NUM_SERVOS];
#endif
#endif

extern Stepper motor;
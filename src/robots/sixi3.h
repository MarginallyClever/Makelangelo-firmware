#pragma once
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == SIXI3

#include "macros.h"

#  define MACHINE_STYLE_NAME       "SIXI3"
#  define MACHINE_HARDWARE_VERSION 1

#  define STEP_DELAY (50)  // delay between steps, in milliseconds, when doing fixed tasks like homing

#  define MAX_SEGMENTS (8)  // override the default to save RAM

//#define MACHINE_HAS_LIFTABLE_PEN
// servo angles for pen control
//#define PEN_UP_ANGLE         (90)
//#define PEN_DOWN_ANGLE       (50)  // Some steppers don't like 0 degrees

#  define NUM_AXIES  (7)  // could be more?
#  define NUM_MOTORS (6)
#  define NUM_SERVOS (1)
#  define NUM_TOOLS  (0)

#  define MAX_FEEDRATE     (100.0)  // depends on timer interrupt & hardware
#  define MIN_FEEDRATE     (0.0)
#  define DEFAULT_FEEDRATE (5.0)

#  define MAX_ACCELERATION     (2500.0)
#  define MIN_ACCELERATION     (0.0)
#  define DEFAULT_ACCELERATION (200.0)

#  define MAX_JERK   (30.0)
#  define MAX_JERK_DEFAULT (20.0)
#  define MAX_Z_JERK (5.0)
#  define MAX_Z_JERK_DEFAULT (0.3)

#define MAX_STEP_RATE_DEFAULT 160

// plan long moves as a set of submoves to increase accuracy.  Uncomment to turn this off.
//#define SUBDIVIDE_LINES
#  define SEGMENTS_PER_SECOND (10)

// It takes STEPS_PER_UNIT steps for the actuator to make one degree.

// 200 motor steps per full turn (360/1.8 deg=200 steps;360/0.9 deg=400 steps)
#  define STEPPER_MOTOR_STEPS_PER_FULL_TURN (200.0)
// *54 input pulley of gearbox has 54 teeth
// /20 output pulley of stepper motor
#  define PULLEY_RATIO (54.0/20.0)
// *70 -to-one gearbox ratio
#  define GEARBOX_RATIO (70.0)
// which means
#  define STEPS_PER_FULL_TURN (STEPPER_MOTOR_STEPS_PER_FULL_TURN * MICROSTEPS * PULLEY_RATIO * GEARBOX_RATIO)
// which means... 105, actually.
#  define STEPS_PER_DEGREE (STEPS_PER_FULL_TURN/360.0)

// the value the rest of the code cares about.
#  define STEPS_PER_UNIT STEPS_PER_DEGREE  // 105
#  define UNITS_PER_STEP (1.0/STEPS_PER_UNIT)  // 0.00952380952?

extern void factory_reset();

#endif  // MACHINE_STYLE == SIXI3

#pragma once
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == STEWART_LINEAR

#include "macros.h"

#  define MACHINE_STYLE_NAME       "STEWART PLATFORM LINEAR"
#  define MACHINE_HARDWARE_VERSION 5

// servo angles for pen control
#  define PEN_UP_ANGLE   (90)
#  define PEN_DOWN_ANGLE (50)  // Some steppers don't like 0 degrees

#  define NUM_TOOLS  (0)
#  define NUM_AXIES  (6)
#  define NUM_MOTORS (6)
#  define NUM_SERVOS (0)

#  define MAX_FEEDRATE     (80.0f)  // depends on timer interrupt & hardware
#  define MIN_FEEDRATE     (0.0f)
#  define DEFAULT_FEEDRATE (10.0f)
#define MAX_STEP_RATE_DEFAULT (160.0f)

#  define MAX_ACCELERATION     (1000.0f)
#  define MIN_ACCELERATION     (0.0f)
#  define DEFAULT_ACCELERATION (500.0f)

#  define MAX_JERK (50.0)
#  define MAX_JERK_DEFAULT (5.0)

#  define MAX_SEGMENTS (16)

// plan long moves as a set of submoves to increase accuracy.  Uncomment to turn this off.
#  define SUBDIVIDE_LINES
#  define SEGMENTS_PER_SECOND (10)

// top center to wrist hole (relative): X7.635 Y+/-0.553 Z0.87
#  define EE_X ( 36.742)
#  define EE_Y (  7.500)
#  define EE_Z (-24.000)
// base center to shoulder hole (relative): X8.093 Y+/-2.15 Z7.831
#  define BASE_X (8.093)
#  define BASE_Y (2.150)
#  define BASE_Z (6.618)

// It takes STEPS_PER_UNIT steps for the actuator to move 1mm.

// 200 motor steps per full turn (360/1.8 deg=200 steps;360/0.9 deg=400 steps)
#  define STEPPER_MOTOR_STEPS_PER_FULL_TURN (200.0)
#  define TURNS_PER_INCH                    (20.0)
#  define STEPS_PER_INCH                    (STEPPER_MOTOR_STEPS_PER_FULL_TURN * MICROSTEPS * TURNS_PER_INCH)
#  define MM_PER_INCH                       (25.4)
#  define STEPS_PER_MM                      (STEPS_PER_INCH / MM_PER_INCH)

// the value the rest of the code cares about.
#  define STEPS_PER_UNIT STEPS_PER_MM
#  define UNITS_PER_STEP (1.0/STEPS_PER_UNIT)

#define MAX_TRAVEL 100
#define MIN_TRAVEL 0

extern void stewartDemo();

extern void factory_reset();
FORCE_INLINE void robotMeanwhile() {}

#endif  // #ifdef STEWART_LINEAR

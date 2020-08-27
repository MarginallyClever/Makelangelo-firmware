#pragma once
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------


#if MACHINE_STYLE == ZARPLOTTER

#define MACHINE_STYLE_NAME       "ZARPLOTTER"
#define MACHINE_HARDWARE_VERSION 6
#define MACHINE_HAS_LIFTABLE_PEN

#define NUM_AXIES            (3)  // more motors than axies. Unusual!
#define NUM_MOTORS           (4)
#define NUM_SERVOS           (1)
#define NUM_TOOLS            (1)

#define MAX_FEEDRATE         (200.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (0.0)
#define DEFAULT_FEEDRATE     (90.0)

#define MAX_ACCELERATION     (500.0)
#define MIN_ACCELERATION     (0.0)
#define DEFAULT_ACCELERATION (180.0)

#define MAX_JERK             (10.0)
#define MAX_Z_JERK           (0.3)

#define ZARPLOTTER_MOTOR_SIZE   (45.0f) // mm
#define ZARPLOTTER_COMPENSATION ( + ZARPLOTTER_MOTOR_SIZE)

// plan long moves as a set of submoves to increase accuracy.  Uncomment to turn this off.
#define SUBDIVIDE_LINES
// what is the maximum length of a subdivided line?
#define SEGMENT_MAX_LENGTH_MM  (2)

// servo angles for pen control
#define PEN_UP_ANGLE         (50)
#define PEN_DOWN_ANGLE       (90)  // Some steppers don't like 0 degrees

#define MAX_SEGMENTS         (16)  // override the default to save RAM
#define SEGMENTS_PER_SECOND  (40)

// 400 step-per-turn motors move 0.9 degrees per step.  (360/400=0.9).  Marginallyclever.com default.
// 200 step-per-turn motors move 1.8 degrees per step.  (360/200=1.8)
// see your stepper motor data sheet for more info.
#ifndef DEGREES_PER_STEP
#define DEGREES_PER_STEP     (1.8)
#endif

// stepper motor drivers can use microstepping to split steps into fractions of steps for greater precision.
// A4988 drivers (Marginallyclever.com default) use 16x.
// DRV8825 can go up to 32x.
// TMC2130 can go to 256x.
// see your driver data sheet.
// note that some boards have dip switches or jumpers that can be activated to turn stepping on and off.
// make sure your dip switch settings match the firmware value.
#ifndef MICROSTEPS
#define MICROSTEPS           (16.0)
#endif

// Marginallyclever.com uses GT2 timing belt, which has 2mm teeth.
// We also use GT2-20 pulleys which have 20 teeth.
// 20*2 means the pitch is 40.
#define PULLEY_PITCH         (40.0)

// These numbers are calculated from the above.  No need to change these.
#ifndef NORMAL_MOTOR_STEPS
#define NORMAL_MOTOR_STEPS   (360.0/DEGREES_PER_STEP)
#endif
#define STEPS_PER_TURN       (NORMAL_MOTOR_STEPS * MICROSTEPS)
#define MM_PER_STEP          (PULLEY_PITCH/STEPS_PER_TURN)

#endif  // ZARPLOTTER

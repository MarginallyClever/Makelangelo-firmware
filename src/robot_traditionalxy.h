#pragma once
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == TRADITIONALXY

#define MACHINE_STYLE_NAME       "TRADITIONALXY"
#define MACHINE_HARDWARE_VERSION 1
#define MACHINE_HAS_LIFTABLE_PEN

#define STEP_DELAY           (50)  // delay between steps, in milliseconds, when doing fixed tasks like homing

// servo angles for pen control
#define PEN_UP_ANGLE         (90)
#define PEN_DOWN_ANGLE       (50)  // Some steppers don't like 0 degrees

#ifndef NUM_AXIES
#define NUM_AXIES            (3)  // could be more?
#endif

#define NUM_MOTORS           (3)
#define NUM_SERVOS           (1)
#define NUM_TOOLS            (1)

#define MAX_FEEDRATE         (100.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (0.0)
#define DEFAULT_FEEDRATE     (90.0)

#define MAX_ACCELERATION     (500.0)
#define MIN_ACCELERATION     (0.0)
#define DEFAULT_ACCELERATION (180.0)

#define MAX_JERK             (10.0)
#define MAX_Z_JERK           (0.3)

// plan long moves as a set of submoves to increase accuracy.  Uncomment to turn this off.
//#define SUBDIVIDE_LINES
#define SEGMENTS_PER_SECOND (10)

// alter these two values to equal gear ratio * motor steps * microstepping for each axis.
// by default they are set the same as all other robots in the system.
#define MM_PER_STEP_X    MM_PER_STEP
#define MM_PER_STEP_Y    MM_PER_STEP

//#define NORMAL_MOTOR_STEPS   200  // 1.8 degrees per step
#define NORMAL_MOTOR_STEPS   400  // 0.9 degrees per step


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


#if NORMAL_MOTOR_STEPS == 200
#define DEFAULT_FEEDRATE     (180.0)
#define DEFAULT_ACCELERATION (150.0)
#define DEGREES_PER_STEP     (1.8)
#endif
#if NORMAL_MOTOR_STEPS == 400
#define DEFAULT_FEEDRATE     (100.0)
#define DEFAULT_ACCELERATION (150.0)
#define DEGREES_PER_STEP     (0.9)
#endif


// These numbers are calculated from the above.  No need to change these.
#ifndef NORMAL_MOTOR_STEPS
#define NORMAL_MOTOR_STEPS   (360.0/DEGREES_PER_STEP)
#endif
#define STEPS_PER_TURN       (NORMAL_MOTOR_STEPS * MICROSTEPS)
#define MM_PER_STEP          (PULLEY_PITCH/STEPS_PER_TURN)
#define STEPS_PER_MM         (STEPS_PER_TURN/PULLEY_PITCH)
#define MICROSTEP_PER_DEGREE (STEPS_PER_TURN/360.0)


#endif  // #ifdef TRADITIONALXY

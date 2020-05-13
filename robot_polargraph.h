#pragma once
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == POLARGRAPH

#define MACHINE_STYLE_NAME       "POLARGRAPH"

// supported versions of makelangelo polargraph robot
#define MAKELANGELO_3    3
#define MAKELANGELO_3_3  4  // Makelangelo Huge
#define MAKELANGELO_5    5
#define MAKELANGELO_6    6  // for testing

#define MACHINE_HARDWARE_VERSION   MAKELANGELO_5  // Change me


// choose one of the following
//#define NORMAL_MOTOR_STEPS   200  // 1.8 degrees per step
#define NORMAL_MOTOR_STEPS   400  // 0.9 degrees per step


#define MACHINE_HAS_LIFTABLE_PEN

// what is the maximum length of a subdivided line?
#define SEGMENTS_PER_SECOND  (40)


// servo angles for pen control
#define PEN_UP_ANGLE         (90)
#define PEN_DOWN_ANGLE       (40)  // Some steppers don't like 0 degrees

#define NUM_AXIES            (3)
#define NUM_MOTORS           (2)
#define NUM_SERVOS           (1)
#define NUM_TOOLS            (1)

#define MAX_FEEDRATE         (1000.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (0.0)

#define MAX_FEEDRATE_Z       (2000.0)  // depends on timer interrupt & hardware

#define MAX_ACCELERATION     (2000.0)
#define MIN_ACCELERATION     (0.0)

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

#define MAX_JERK             (8.0)
#define MAX_JERK_Z           (0.3)

// uncomment this line to adjust acceleration based on pen position
#define DYNAMIC_ACCELERATION

#if MACHINE_HARDWARE_VERSION == MAKELANGELO_3
#define MAX_SEGMENTS         (32)
#define HAS_SD
#define HAS_LCD
#endif
#if MACHINE_HARDWARE_VERSION == MAKELANGELO_3_3
#define MAX_SEGMENTS         (32)
#define USE_LIMIT_SWITCH
#define HAS_SD
#define HAS_LCD
#endif
#if MACHINE_HARDWARE_VERSION == MAKELANGELO_5
#define MAX_SEGMENTS         (32)
#define USE_LIMIT_SWITCH
#define HAS_SD
#define HAS_LCD
#endif
#if MACHINE_HARDWARE_VERSION == MAKELANGELO_6
#define MAX_SEGMENTS         (16)
#define USE_LIMIT_SWITCH
#define HAS_TMC2130  // driver type
#define STEALTHCHOP  // quiet operation
#define HAS_SD
#define HAS_LCD
#endif

#if defined(HAS_TMC2130) || defined(USE_LIMIT_SWITCH)
#define CAN_HOME
#endif

//------------------------------------------------------------------------------
// MOTOR DETAILS
//------------------------------------------------------------------------------

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
#define STEPS_PER_MM         (STEPS_PER_TURN/PULLEY_PITCH)
#define MICROSTEP_PER_DEGREE (STEPS_PER_TURN/360.0)

//------------------------------------------------------------------------------

//extern void calibrateBelts();
extern void recordHome();

// convert belt length to cartesian position, save that as home pos.
extern void calibrationToPosition();

extern void makelangelo6Setup();

extern void makelangelo5Setup();

extern void makelangelo33Setup();


#endif  // #ifdef POLARGRAPH

#pragma once
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == STEWART

#define MACHINE_STYLE_NAME           "STEWART"
#define MACHINE_HARDWARE_VERSION     5

// servo angles for pen control
#define PEN_UP_ANGLE         (90)
#define PEN_DOWN_ANGLE       (50)  // Some steppers don't like 0 degrees

#define NUM_TOOLS            (1)
#define NUM_AXIES            (6)
#define NUM_MOTORS           (6)
#define NUM_SERVOS           (0)

#define MAX_FEEDRATE         (200.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (0)
#define DEFAULT_FEEDRATE     (60.0)

#define MAX_ACCELERATION     (1000)
#define MIN_ACCELERATION     (0)
#define DEFAULT_ACCELERATION (500)

#define MAX_JERK             (5.0)

#define MAX_SEGMENTS         (16)

// plan long moves as a set of submoves to increase accuracy.  Uncomment to turn this off.
#define SUBDIVIDE_LINES
#define SEGMENTS_PER_SECOND (10)

#define BICEP_LENGTH         ( 5.000)
#define FOREARM_LENGTH       (16.750)
#define SWITCH_ANGLE         (90-18.690)
// top center to wrist hole (relative): X7.635 Y+/-0.553 Z0.87
#define T2W_X                ( 7.635)
#define T2W_Y                ( 0.553)
#define T2W_Z                (-0.870)
// base center to shoulder hole (relative): X8.093 Y+/-2.15 Z7.831
#define B2S_X                ( 8.093)
#define B2S_Y                ( 2.150)
#define B2S_Z                ( 6.618)


// choose one of the following
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


extern void stewartDemo();

#endif  // #ifdef STEWART

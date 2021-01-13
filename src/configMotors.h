#pragma once
/*
 * Motor settings shared between various kinematic systems
 */


// choose one of the following (if not already defined in robot_<x>.h)
#ifndef NORMAL_MOTOR_STEPS
    //#define NORMAL_MOTOR_STEPS   200  // 1.8 degrees per step
    #define NORMAL_MOTOR_STEPS   400  // 0.9 degrees per step
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


//if default steps are not defined in in robot_<x>.h, safe-define them here
#if NORMAL_MOTOR_STEPS == 200
    #ifndef DEFAULT_FEEDRATE
        #define DEFAULT_FEEDRATE     (180.0)
    #endif
    #ifndef DEFAULT_ACCELERATION
        #define DEFAULT_ACCELERATION (150.0)
    #endif
    #ifndef DEGREES_PER_STEP
        #define DEGREES_PER_STEP     (1.8)
    #endif
#endif
#if NORMAL_MOTOR_STEPS == 400
    #ifndef DEFAULT_FEEDRATE
        #define DEFAULT_FEEDRATE     (100.0)
    #endif
    #ifndef DEFAULT_ACCELERATION
        #define DEFAULT_ACCELERATION (150.0)
    #endif
    #ifndef DEGREES_PER_STEP
        #define DEGREES_PER_STEP     (0.9)
    #endif
#endif


// These numbers are calculated from the above.  No need to change these. Only define them if they are not already defined in in robot_<x>.h
#ifndef NORMAL_MOTOR_STEPS
    #define NORMAL_MOTOR_STEPS   (360.0/DEGREES_PER_STEP)
#endif
#ifndef MM_PER_STEP
    #define MM_PER_STEP          (PULLEY_PITCH/STEPS_PER_TURN)
#endif

#define STEPS_PER_TURN       (NORMAL_MOTOR_STEPS * MICROSTEPS)
#define STEPS_PER_MM         (STEPS_PER_TURN/PULLEY_PITCH)
#define MICROSTEP_PER_DEGREE (STEPS_PER_TURN/360.0)

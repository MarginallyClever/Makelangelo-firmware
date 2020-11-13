#pragma once
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == DELTA

#define MACHINE_STYLE_NAME           "DELTA"
#define MACHINE_HARDWARE_VERSION     5


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


// known machine configurations
#define DELTA_STYLE_MARGINALLYCLEVER_V8  1
#define DELTA_STYLE_JUDAH                2

#define DELTA_STYLE DELTA_STYLE_JUDAH  // change this


#define STEP_DELAY           (50)  // delay between steps, in milliseconds, when doing fixed tasks like homing

// plan long moves as a set of submoves to increase accuracy.  Uncomment to turn this off.
#define SUBDIVIDE_LINES
#define SEGMENTS_PER_SECOND (10)

// servo angles for pen control
#define PEN_UP_ANGLE         (90)
#define PEN_DOWN_ANGLE       (50)  // Some steppers don't like 0 degrees


#define NUM_TOOLS            (1)

#define MAX_FEEDRATE         (100.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (0.0)
#define DEFAULT_FEEDRATE     (90.0)

#define MAX_ACCELERATION     (500.0)
#define MIN_ACCELERATION     (0.0)
#define DEFAULT_ACCELERATION (180.0)

#define MAX_JERK             (10.0)
#define MAX_Z_JERK           (0.3)


#if DELTA_STYLE  == DELTA_STYLE_MARGINALLYCLEVER_V8
// physical measurements of the machine
#define CENTER_TO_SHOULDER       ( 3.770f)  // cm (f)
#define SHOULDER_TO_ELBOW        ( 5.000f)  // cm (Rf)
#define ELBOW_TO_WRIST           (16.500f)  // cm (Re)
#define EFFECTOR_TO_WRIST        ( 1.724f)  // cm (e)
#define CENTER_TO_FLOOR          (18.900f)  // cm
#define DEGREES_ABOVE_HORIZONTAL (24.000f)
#define NUM_AXIES                (3)
#define NUM_MOTORS               (3)
#define NUM_SERVOS               (0)
#define DELTA_HOME_DIRECTION     LOW  // LOW or HIGH
#endif


#if DELTA_STYLE  == DELTA_STYLE_JUDAH
// physical measurements of the machine
#define CENTER_TO_SHOULDER       (12.65682f)  // cm (f)
#define SHOULDER_TO_ELBOW        (20.320f)  // cm (Rf)
#define ELBOW_TO_WRIST           (55.820f)  // cm (Re)
#define EFFECTOR_TO_WRIST        ( 1.724f)  // cm (E)
#define CENTER_TO_FLOOR          (60.000f)  // cm
#define DEGREES_ABOVE_HORIZONTAL (24.000f)
#define NUM_AXIES                (4)
#define NUM_MOTORS               (4)
#define NUM_SERVOS               (0)
#define DELTA_HOME_DIRECTION     LOW  // LOW or HIGH
#endif

#endif  // MACHINE_STYLE == DELTA

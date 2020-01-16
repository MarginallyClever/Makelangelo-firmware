#ifndef ROBOT_POLARGRAPH_H
#define ROBOT_POLARGRAPH_H
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == POLARGRAPH

#define MACHINE_STYLE_NAME       "POLARGRAPH"

// supported versions of makelangelo polargraph robot
#define MAKELANGELO_3    3
#define MAKELANGELO_3_3  4
#define MAKELANGELO_5    5
#define MAKELANGELO_6    6  // for testing


/// -------------- change here ----------------
// change this line for your version
#define MACHINE_HARDWARE_VERSION   MAKELANGELO_5

// choose one of the following
#define NORMAL_MOTOR_STEPS   200  // 1.8 degrees per step
//#define NORMAL_MOTOR_STEPS   400  // 0.9 degrees per step
/// -------------- change here ----------------


#define MACHINE_HAS_LIFTABLE_PEN

// what is the maximum length of a subdivided line?
#define SEGMENTS_PER_SECOND (2)


// servo angles for pen control
#define PEN_UP_ANGLE         (90)
#define PEN_DOWN_ANGLE       (30)  // Some steppers don't like 0 degrees

#define NUM_AXIES            (3)
#define NUM_MOTORS           (2)
#define NUM_SERVOS           (1)
#define NUM_TOOLS            (1)

#define MAX_FEEDRATE         (500.0)  // depends on timer interrupt & hardware
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
#define MAX_JERK_Z           (0.0)

// uncomment this line to adjust acceleration based on pen position
#define DYNAMIC_ACCELERATION

#if MACHINE_HARDWARE_VERSION == MAKELANGELO_3
#define MAX_SEGMENTS         (16)
#define HAS_SD
#define HAS_LCD
#endif
#if MACHINE_HARDWARE_VERSION == MAKELANGELO_3_3
#define MAX_SEGMENTS         (16)
#define USE_LIMIT_SWITCH
#define HAS_SD
#define HAS_LCD
#endif
#if MACHINE_HARDWARE_VERSION == MAKELANGELO_5
#define MAX_SEGMENTS         (16)
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

//extern void calibrateBelts();
extern void recordHome();


#endif  // #ifdef POLARGRAPH


#endif  // #ifndef ROBOT_POLARGRAPH_H

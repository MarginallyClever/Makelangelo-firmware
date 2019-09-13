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

/// -------------- change here ----------------
// change this line for your version
#define MACHINE_HARDWARE_VERSION   MAKELANGELO_5

// choose one of the following
#define NORMAL_MOTOR_STEPS   200
//#define NORMAL_MOTOR_STEPS   400
/// -------------- change here ----------------


#define MACHINE_HAS_LIFTABLE_PEN

// plan long moves as a set of submoves to increase accuracy.  Uncomment to turn this off.
#define SUBDIVIDE_LINES
// what is the maximum length of a subdivided line?
#define SEGMENT_MAX_LENGTH_CM  (1)

// servo angles for pen control
#define PEN_UP_ANGLE         (90)
#define PEN_DOWN_ANGLE       (50)  // Some steppers don't like 0 degrees

#define NUM_AXIES            (3)
#define NUM_MOTORS           (2)
#define NUM_SERVOS           (1)
#define NUM_TOOLS            (1)

#define MAX_FEEDRATE         (100.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (0.0)

#define MAX_ACCELERATION     (2000.0)
#define MIN_ACCELERATION     (0.0)

#if NORMAL_MOTOR_STEPS == 200
#define DEFAULT_FEEDRATE     (110.0)
#define DEFAULT_ACCELERATION (1800.0)
#define DEGREES_PER_STEP     (1.8)
#endif
#if NORMAL_MOTOR_STEPS == 400
#define DEFAULT_FEEDRATE     (60.0)
#define DEFAULT_ACCELERATION (900.0)
#define DEGREES_PER_STEP     (0.9)
#endif

#define DYNAMIC_ACCELERATION  // uncomment this line to adjust acceleration based on pen position

#define MAX_JERK             (5.0)
#define MAX_Z_JERK           (0.3)

#if MACHINE_HARDWARE_VERSION == MAKELANGELO_5
#define MAX_SEGMENTS         (16)  // has LCD, needs more ram.
#define USE_LIMIT_SWITCH
#define HAS_SD
#define HAS_LCD
#endif
#if MACHINE_HARDWARE_VERSION == MAKELANGELO_3
#define MAX_SEGMENTS         (16)  // has LCD, needs more ram.
#define HAS_SD
#define HAS_LCD
#endif
#if MACHINE_HARDWARE_VERSION == MAKELANGELO_3_3
#define MAX_SEGMENTS         (16)  // has LCD, needs more ram.
#define USE_LIMIT_SWITCH
#define HAS_SD
#define HAS_LCD
#endif

extern void calibrateBelts();
extern void recordHome();


#endif  // #ifdef POLARGRAPH


#endif  // #ifndef ROBOT_POLARGRAPH_H

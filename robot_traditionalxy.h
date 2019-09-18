#ifndef ROBOT_TRADITIONALXY_H
#define ROBOT_TRADITIONALXY_H
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

#define NUM_AXIES            (3)  // could be more?
#define NUM_MOTORS           (2)
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
// what is the maximum length of a subdivided line?
#define SEGMENT_MAX_LENGTH_MM  (20)

// alter these two values to equal gear ratio * motor steps * microstepping for each axis.
// by default they are set the same as all other robots in the system.
#define MM_PER_STEP_X    MM_PER_STEP
#define MM_PER_STEP_Y    MM_PER_STEP

#endif  // #ifdef TRADITIONALXY


#endif  // #ifndef ROBOT_TRADITIONALXY_H

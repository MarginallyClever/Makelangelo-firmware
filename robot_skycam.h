#ifndef ROBOT_SKYCAM_H
#define ROBOT_SKYCAM_H
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == SKYCAM

#define MACHINE_STYLE_NAME       "SKYCAM"
#define MACHINE_HARDWARE_VERSION 6

#define NUM_AXIES            (3)  // more motors than axies. Unusual!
#define NUM_MOTORS           (4)
#define NUM_SERVOS           (0)
#define NUM_TOOLS            (1)

#define MAX_FEEDRATE         (100.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (0.0)
#define DEFAULT_FEEDRATE     (90.0)

#define MAX_ACCELERATION     (500.0)
#define MIN_ACCELERATION     (0.0)
#define DEFAULT_ACCELERATION (180.0)

#define MAX_JERK             (10.0)
#define MAX_Z_JERK           (0.3)

#define SKYCAM_MOTOR_SIZE   (4.5f)
#define SKYCAM_PLOTTER_SIZE (6.0f)
#define SKYCAM_COMPENSATION (SKYCAM_PLOTTER_SIZE/2.0f + SKYCAM_MOTOR_SIZE)

// plan long moves as a set of submoves to increase accuracy.  Uncomment to turn this off.
#define SUBDIVIDE_LINES
// what is the maximum length of a subdivided line?
#define SEGMENT_MAX_LENGTH_MM  (2)

// servo angles for pen control
#define PEN_UP_ANGLE         (50)
#define PEN_DOWN_ANGLE       (90)  // Some steppers don't like 0 degrees

#endif  // SKYCAM


#endif  // #ifndef ROBOT_SKYCAM_H

#pragma once
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

// the machine size is a box X wide Y deep and Z tall.
// the home position is in the bottom center of the box.
//
// a--b      g--h
// |  \      /  |  
// |    \    /  |
// |      \  /  |
// |       fff  |
// |            |
// c------d-----e

// motor size is the a-b distance, where b is the last point that the belt is touched.
// this measure is equal on both X and Y.
#define SKYCAM_MOTOR_SIZE   (4.5f)

// SKYCAM_PLOTTER_SIZE is the diameter of the tool (f)
#define SKYCAM_PLOTTER_SIZE (6.0f)

// In order to try and correct bowing effect for the motor size and plotter size, we find this number one time.
// it's assumed to be the same for all motors.
#define SKYCAM_COMPENSATION (SKYCAM_PLOTTER_SIZE/2.0f + SKYCAM_MOTOR_SIZE)

// plan long moves as a set of submoves to increase accuracy.  Uncomment to turn this off.
#define SUBDIVIDE_LINES
#define SEGMENTS_PER_SECOND (10)

// servo angles for pen control
#define PEN_UP_ANGLE         (50)
#define PEN_DOWN_ANGLE       (90)  // Some steppers don't like 0 degrees

#endif  // SKYCAM

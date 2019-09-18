#ifndef ROBOT_ZARPLOTTER_H
#define ROBOT_ZARPLOTTER_H
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------


#if MACHINE_STYLE == ZARPLOTTER

#define MACHINE_STYLE_NAME       "ZARPLOTTER"
#define MACHINE_HARDWARE_VERSION 6
#define MACHINE_HAS_LIFTABLE_PEN

#define NUM_AXIES            (3)  // more motors than axies. Unusual!
#define NUM_MOTORS           (4)
#define NUM_SERVOS           (1)
#define NUM_TOOLS            (1)

#define MAX_FEEDRATE         (200.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (0.0)
#define DEFAULT_FEEDRATE     (90.0)

#define MAX_ACCELERATION     (500.0)
#define MIN_ACCELERATION     (0.0)
#define DEFAULT_ACCELERATION (180.0)

#define MAX_JERK             (10.0)
#define MAX_Z_JERK           (0.3)

#define ZARPLOTTER_MOTOR_SIZE   (45.0f) // mm
#define ZARPLOTTER_PLOTTER_SIZE (60.0f) // mm
#define ZARPLOTTER_COMPENSATION (ZARPLOTTER_PLOTTER_SIZE/2.0f + ZARPLOTTER_MOTOR_SIZE)

// plan long moves as a set of submoves to increase accuracy.  Uncomment to turn this off.
#define SUBDIVIDE_LINES
// what is the maximum length of a subdivided line?
#define SEGMENT_MAX_LENGTH_MM  (2)

// servo angles for pen control
#define PEN_UP_ANGLE         (50)
#define PEN_DOWN_ANGLE       (90)  // Some steppers don't like 0 degrees

#define MAX_SEGMENTS         (16)  // override the default to save RAM

#endif  // ZARPLOTTER


#endif  // #ifndef ROBOT_ZARPLOTTER_H

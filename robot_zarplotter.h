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

#define MAX_FEEDRATE         (15000.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (100)
#define MAX_JERK             (15.0)
#define DEFAULT_FEEDRATE     (10000.0)
#define DEFAULT_ACCELERATION (3500)

#define ZARPLOTTER_MOTOR_SIZE   (4.5f)
#define ZARPLOTTER_PLOTTER_SIZE (6.0f)
#define ZARPLOTTER_COMPENSATION (ZARPLOTTER_PLOTTER_SIZE/2.0f + ZARPLOTTER_MOTOR_SIZE)

#define MAX_ACCELERATION     (5000)
#define MIN_ACCELERATION     (100)

#define SUBDIVIDE_LINES
#define SEGMENT_PER_CM_LINE  (2)  // lines are split into segments.  How long are the segments?
#define SEGMENT_PER_CM_ARC   (3)  // Arcs are split into segments.  How long are the segments?

// servo angles for pen control
#define PEN_UP_ANGLE         (50)
#define PEN_DOWN_ANGLE       (90)  // Some steppers don't like 0 degrees

#endif  // ZARPLOTTER


#endif  // #ifndef ROBOT_ZARPLOTTER_H

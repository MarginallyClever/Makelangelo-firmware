#ifndef ROBOT_POLARGRAPH_H
#define ROBOT_POLARGRAPH_H
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == POLARGRAPH

#define MACHINE_STYLE_NAME       "POLARGRAPH"
//#define MACHINE_HARDWARE_VERSION 3  // If you have a makelangelo 3+
#define MACHINE_HARDWARE_VERSION 5  // If you have a makelangelo 5+
#define MACHINE_HAS_LIFTABLE_PEN

#define SUBDIVIDE_LINES
#define SEGMENT_PER_CM_LINE  (2)  // lines are subdivided.  How long are the divisions?
#define SEGMENT_PER_CM_ARC   (3)  // Arcs are subdivided.  How long are the divisions?

// servo angles for pen control
#define PEN_UP_ANGLE         (90)
#define PEN_DOWN_ANGLE       (50)  // Some steppers don't like 0 degrees

#define NUM_AXIES            (3)
#define NUM_MOTORS           (2)
#define NUM_SERVOS           (1)
#define NUM_TOOLS            (1)

#define MAX_FEEDRATE         (20000.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (100)
#define DEFAULT_FEEDRATE     (9000.0)

#define MAX_ACCELERATION     (5000)
#define MIN_ACCELERATION     (100)
#define DEFAULT_ACCELERATION (2500)

#define MAX_JERK             (5.0)


#if MACHINE_HARDWARE_VERSION == 5
#define USE_LIMIT_SWITCH     (1)
#define HAS_SD
#define HAS_LCD
#endif
#if MACHINE_HARDWARE_VERSION == 3
#define HAS_SD
#define HAS_LCD
#endif



#endif  // #ifdef POLARGRAPH


#endif  // #ifndef ROBOT_POLARGRAPH_H

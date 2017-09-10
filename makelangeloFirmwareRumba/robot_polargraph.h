#ifndef ROBOT_POLARGRAPH_H
#define ROBOT_POLARGRAPH_H
//------------------------------------------------------------------------------
// Makelangelo - a mural drawing robot
// dan@marginallycelver.com 2013-12-26
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/Makelangelo for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == POLARGRAPH

//#define MAKELANGELO_HARDWARE_VERSION 3  // If you have a makelangelo 3+
#define MAKELANGELO_HARDWARE_VERSION 5  // If you have a makelangelo 5+


#define STEP_DELAY           (50)  // delay between steps, in milliseconds, when doing fixed tasks like homing

#define MAX_ACCELERATION     (5000)
#define MIN_ACCELERATION     (100)
// servo angles for pen control
#define PEN_UP_ANGLE         (90)
#define PEN_DOWN_ANGLE       (50)  // Some steppers don't like 0 degrees


#define NUM_MOTORS           (2)
#define NUM_SERVOS           (1)
#define NUM_TOOLS            (1)

#define MAX_FEEDRATE         (9000.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (100)
#define MAX_JERK             (5.0)
#define DEFAULT_FEEDRATE     (7000.0)
#define DEFAULT_ACCELERATION (2500)


#if MAKELANGELO_HARDWARE_VERSION == 5
#define USE_LIMIT_SWITCH    (1)  // Comment out this line to disable findHome and limit switches
#define HAS_SD                   // comment this out if there is no SD card
#define HAS_LCD                  // comment this out if there is no SMART LCD controller
#endif
#if MAKELANGELO_HARDWARE_VERSION == 3
#define HAS_SD                   // comment this out if there is no SD card
#define HAS_LCD                  // comment this out if there is no SMART LCD controller
#endif



#endif  // #ifdef POLARGRAPH


#endif  // #ifndef ROBOT_MAKELANGELO_H


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

#define MAX_ACCELERATION     (5000)
#define MIN_ACCELERATION     (100)

// servo angles for pen control
#define PEN_UP_ANGLE         (90)
#define PEN_DOWN_ANGLE       (50)  // Some steppers don't like 0 degrees

#define NUM_AXIES            (3)  // could be more?
#define NUM_MOTORS           (2)
#define NUM_SERVOS           (1)
#define NUM_TOOLS            (1)

#define MAX_FEEDRATE         (40000.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (100)
#define MAX_JERK             (5.0)
#define DEFAULT_FEEDRATE     (15000.0)
#define DEFAULT_ACCELERATION (2500)

//#define SUBDIVIDE_LINES
#define SEGMENT_PER_CM_LINE  (1)  // lines are subdivided.  How long are the divisions?
#define SEGMENT_PER_CM_ARC   (3)  // Arcs are subdivided.  How long are the divisions?


#endif  // #ifdef TRADITIONALXY


#endif  // #ifndef ROBOT_TRADITIONALXY_H

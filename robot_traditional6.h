#pragma once
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == TRADITIONAL6

#define MACHINE_STYLE_NAME       "TRADITIONAL6"
#define MACHINE_HARDWARE_VERSION 1

#define STEP_DELAY           (50)  // delay between steps, in milliseconds, when doing fixed tasks like homing

#define MAX_SEGMENTS         (16)  // override the default to save RAM

//#define MACHINE_HAS_LIFTABLE_PEN
// servo angles for pen control
//#define PEN_UP_ANGLE         (90)
//#define PEN_DOWN_ANGLE       (50)  // Some steppers don't like 0 degrees

#define NUM_AXIES            (6)  // could be more?
#define NUM_MOTORS           (6)
#define NUM_SERVOS           (0)
#define NUM_TOOLS            (0)

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
#define SEGMENTS_PER_SECOND (10)

// this value should equal gear ratio * motor steps * microstepping for each axis.
// by default this is set the same for all motors, and you'll probably want to break it out into separate value per-axis.
#define MM_PER_STEP  (1 * 200 * 2)  // 1:1 ratio * 200 steps per turn * 1/2 stepping

#endif  // #ifdef TRADITIONAL6

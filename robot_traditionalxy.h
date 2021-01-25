#pragma once
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

#ifndef NUM_AXIES
#define NUM_AXIES            (3)  // could be more?
#endif

#ifndef NUM_MOTORS
#define NUM_MOTORS           (3)
#endif

#ifndef NUM_SERVOS
#define NUM_SERVOS           (1)
#endif

#ifndef NUM_TOOLS
#define NUM_TOOLS            (1)
#endif

#ifndef MAX_FEEDRATE
#define MAX_FEEDRATE         (100.0)  // depends on timer interrupt & hardware
#endif

#ifndef MIN_FEEDRATE
#define MIN_FEEDRATE         (0.0)
#endif

#ifndef DEFAULT_FEEDRATE
#define DEFAULT_FEEDRATE     (90.0)
#endif

#ifndef MAX_ACCELERATION
#define MAX_ACCELERATION     (500.0)
#endif

#define MIN_ACCELERATION     (0.0)

#ifndef DEFAULT_ACCELERATION
#define DEFAULT_ACCELERATION (180.0)
#endif

#define MAX_JERK             (10.0)
#define MAX_Z_JERK           (0.3)

#define SEGMENTS_PER_SECOND (5)

// alter these two values to equal gear ratio * motor steps * microstepping for each axis.
// by default they are set the same as all other robots in the system.
#ifndef MM_PER_STEP_X
#define MM_PER_STEP_X    MM_PER_STEP
#endif

#ifndef MM_PER_STEP_Y
#define MM_PER_STEP_Y    MM_PER_STEP
#endif


#endif  // #ifdef TRADITIONALXY

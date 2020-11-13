#pragma once
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == SCARA

#define MACHINE_STYLE_NAME       "SCARA"
#define MACHINE_HARDWARE_VERSION 1
#define MACHINE_HAS_LIFTABLE_PEN

// bicep is the connection between the base and the elbow.
#define BICEP_LENGTH_MM      (150.0)
// forearm is the connection between the elbow and the pen center
#define FOREARM_LENGTH_MM    (110.0)


#define STEP_DELAY           (50)  // delay between steps, in milliseconds, when doing fixed tasks like homing

// servo angles for pen control
#define PEN_UP_ANGLE         (90)
#define PEN_DOWN_ANGLE       (50)  // Some steppers don't like 0 degrees

#ifndef NUM_AXIES
#define NUM_AXIES            (3)  // could be more?
#endif

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
#define SEGMENTS_PER_SECOND  (10)

#endif  // #ifdef TRADITIONALXY

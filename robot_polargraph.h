#pragma once
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == POLARGRAPH

#define MACHINE_STYLE_NAME       "POLARGRAPH"

// supported versions of makelangelo polargraph robot
#define MAKELANGELO_CUSTOM   0  // Your DIY machine
#define MAKELANGELO_3        3
#define MAKELANGELO_3_3      4  // Makelangelo Huge
#define MAKELANGELO_5        5
#define MAKELANGELO_6        6  // for testing


#define MACHINE_HARDWARE_VERSION   MAKELANGELO_5  // Change me

#define MACHINE_HAS_LIFTABLE_PEN

// what is the maximum length of a subdivided line?
#define SEGMENTS_PER_SECOND  (40)


// servo angles for pen control
#define PEN_UP_ANGLE         (90)
#define PEN_DOWN_ANGLE       (40)  // Some steppers don't like 0 degrees

#define NUM_AXIES            (3)
#define NUM_MOTORS           (2)
#define NUM_SERVOS           (1)
#define NUM_TOOLS            (1)

#define MAX_FEEDRATE         (1000.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (0.0)

#define MAX_FEEDRATE_Z       (2000.0)  // depends on timer interrupt & hardware

#define MAX_ACCELERATION     (2000.0)
#define MIN_ACCELERATION     (0.0)

#define MAX_JERK             (8.0)
#define MAX_JERK_Z           (0.3)

// uncomment this line to adjust acceleration based on pen position
#define DYNAMIC_ACCELERATION

#if MACHINE_HARDWARE_VERSION == MAKELANGELO_3
#ifndef MAX_SEGMENTS
#define MAX_SEGMENTS         (32)
#endif
#define HAS_SD
#define HAS_LCD
#endif

#if MACHINE_HARDWARE_VERSION == MAKELANGELO_3_3
#ifndef MAX_SEGMENTS
#define MAX_SEGMENTS         (32)
#endif
#define USE_LIMIT_SWITCH
#define HAS_SD
#define HAS_LCD
#endif

#if MACHINE_HARDWARE_VERSION == MAKELANGELO_5
#ifndef MAX_SEGMENTS
#define MAX_SEGMENTS         (32)
#endif
#define USE_LIMIT_SWITCH
#define HAS_SD
#define HAS_LCD
#endif

#if MACHINE_HARDWARE_VERSION == MAKELANGELO_6
#ifndef MAX_SEGMENTS
#define MAX_SEGMENTS         (16)
#endif
#define USE_LIMIT_SWITCH
#define HAS_TMC2130  // driver type
#define STEALTHCHOP  // quiet operation
#define HAS_SD
#define HAS_LCD
#endif

#if defined(HAS_TMC2130) || defined(USE_LIMIT_SWITCH)
#define CAN_HOME
#endif

//------------------------------------------------------------------------------

//extern void calibrateBelts();
extern void recordHome();

// convert belt length to cartesian position, save that as home pos.
extern void calibrationToPosition();

// M503 factory reset
extern void polargraphReset();


#endif  // #ifdef POLARGRAPH

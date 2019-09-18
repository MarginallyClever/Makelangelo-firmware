#ifndef ROBOT_STEWART_H
#define ROBOT_STEWART_H
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == STEWART

#define MACHINE_STYLE_NAME           "STEWART"
#define MACHINE_HARDWARE_VERSION     5

// servo angles for pen control
#define PEN_UP_ANGLE         (90)
#define PEN_DOWN_ANGLE       (50)  // Some steppers don't like 0 degrees

#define NUM_TOOLS            (1)
#define NUM_AXIES            (6)
#define NUM_MOTORS           (6)
#define NUM_SERVOS           (0)

#define MAX_FEEDRATE         (200.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (0)
#define DEFAULT_FEEDRATE     (60.0)

#define MAX_ACCELERATION     (1000)
#define MIN_ACCELERATION     (0)
#define DEFAULT_ACCELERATION (500)

#define MAX_JERK             (5.0)

#define MAX_SEGMENTS         (16)

// plan long moves as a set of submoves to increase accuracy.  Uncomment to turn this off.
#define SUBDIVIDE_LINES
// what is the maximum length of a subdivided line?
#define SEGMENT_MAX_LENGTH_MM  (1)

#define BICEP_LENGTH         ( 5.000)
#define FOREARM_LENGTH       (16.750)
#define SWITCH_ANGLE         (90-18.690)
// top center to wrist hole (relative): X7.635 Y+/-0.553 Z0.87
#define T2W_X                ( 7.635)
#define T2W_Y                ( 0.553)
#define T2W_Z                (-0.870)
// base center to shoulder hole (relative): X8.093 Y+/-2.15 Z7.831
#define B2S_X                ( 8.093)
#define B2S_Y                ( 2.150)
#define B2S_Z                ( 6.618)


extern void stewartDemo();

#endif  // #ifdef STEWART


#endif  // #ifndef ROBOT_STEWART_H

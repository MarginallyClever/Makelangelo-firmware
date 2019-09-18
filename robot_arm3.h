#ifndef ROBOT_ARM3_H
#define ROBOT_ARM3_H
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == ARM3

#define MACHINE_STYLE_NAME       "ARM3"
#define MACHINE_HARDWARE_VERSION 1

#define STEP_DELAY           (50)  // delay between steps, in milliseconds, when doing fixed tasks like homing


// servo angles for pen control
#define PEN_UP_ANGLE         (90)
#define PEN_DOWN_ANGLE       (50)  // Some steppers don't like 0 degrees

#define NUM_AXIES            (3)  // could be more?
#define NUM_MOTORS           (3)
#define NUM_SERVOS           (0)
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
#define SUBDIVIDE_LINES
// what is the maximum length of a subdivided line?
#define SEGMENT_MAX_LENGTH_MM  (10)



// machine dimensions
#define BASE_TO_SHOULDER_X    (5.37)  // measured in solidworks
#define BASE_TO_SHOULDER_Z    (9.55)  // measured in solidworks
#define SHOULDER_TO_ELBOW     (25.0)
#define ELBOW_TO_WRIST        (25.0)
#define WRIST_TO_FINGER       (4.0)
#define FINGER_TO_FLOOR       (0.5)
#define GEAR_RATIO            (5.0)

#define HOME_X                (12.850)
#define HOME_Y                (0)
#define HOME_Z                (22.2)

#endif  // #ifdef ARM3


#endif  // #ifndef ROBOT_ARM3_H

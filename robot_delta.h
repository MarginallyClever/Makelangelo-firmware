#ifndef ROBOT_DELTA_H
#define ROBOT_DELTA_H
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == DELTA

#define MACHINE_STYLE_NAME           "DELTA"
#define MACHINE_HARDWARE_VERSION     5

// known machine configurations
#define DELTA_STYLE_MARGINALLYCLEVER_V8  1
#define DELTA_STYLE_JUDAH                2

#define DELTA_STYLE DELTA_STYLE_JUDAH  // change this


#define STEP_DELAY           (50)  // delay between steps, in milliseconds, when doing fixed tasks like homing

// plan long moves as a set of submoves to increase accuracy.  Uncomment to turn this off.
#define SUBDIVIDE_LINES
// what is the maximum length of a subdivided line?
#define SEGMENT_MAX_LENGTH_MM  (10)

// servo angles for pen control
#define PEN_UP_ANGLE         (90)
#define PEN_DOWN_ANGLE       (50)  // Some steppers don't like 0 degrees


#define NUM_TOOLS            (1)

#define MAX_FEEDRATE         (100.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (0.0)
#define DEFAULT_FEEDRATE     (90.0)

#define MAX_ACCELERATION     (500.0)
#define MIN_ACCELERATION     (0.0)
#define DEFAULT_ACCELERATION (180.0)

#define MAX_JERK             (10.0)
#define MAX_Z_JERK           (0.3)



#if DELTA_STYLE  == DELTA_STYLE_MARGINALLYCLEVER_V8
// physical measurements of the machine
#define CENTER_TO_SHOULDER       ( 3.770f)  // cm (f)
#define SHOULDER_TO_ELBOW        ( 5.000f)  // cm (Rf)
#define ELBOW_TO_WRIST           (16.500f)  // cm (Re)
#define EFFECTOR_TO_WRIST        ( 1.724f)  // cm (e)
#define CENTER_TO_FLOOR          (18.900f)  // cm
#define DEGREES_ABOVE_HORIZONTAL (24.000f)
#define NUM_AXIES                (3)
#define NUM_MOTORS               (3)
#define NUM_SERVOS               (0)
#define DELTA_HOME_DIRECTION     LOW  // LOW or HIGH
#endif


#if DELTA_STYLE  == DELTA_STYLE_JUDAH
// physical measurements of the machine
#define CENTER_TO_SHOULDER       (12.65682f)  // cm (f)
#define SHOULDER_TO_ELBOW        (20.320f)  // cm (Rf)
#define ELBOW_TO_WRIST           (55.820f)  // cm (Re)
#define EFFECTOR_TO_WRIST        ( 1.724f)  // cm (E)
#define CENTER_TO_FLOOR          (60.000f)  // cm
#define DEGREES_ABOVE_HORIZONTAL (24.000f)
#define NUM_AXIES                (4)
#define NUM_MOTORS               (4)
#define NUM_SERVOS               (0)
#define DELTA_HOME_DIRECTION     LOW  // LOW or HIGH
#endif


#endif  // #ifdef DELTA


#endif  // #ifndef ROBOT_DELTA_H

#ifndef ROBOT_DELTA_H
#define ROBOT_DELTA_H
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == DELTA

#define MACHINE_STYLE_NAME           "DELTA"
#define MACHINE_HARDWARE_VERSION 5  // If you have a makelangelo 5+

// known machine configurations
#define DELTA_STYLE_MARGINALLYCLEVER_V8  1
#define DELTA_STYLE_JUDAH                2

#define DELTA_STYLE DELTA_STYLE_JUDAH  // change this


#define STEP_DELAY           (50)  // delay between steps, in milliseconds, when doing fixed tasks like homing

#define MAX_ACCELERATION     (5000)
#define MIN_ACCELERATION     (100)

#define SUBDIVIDE_LINES
#define SEGMENT_PER_CM_LINE  (2)  // lines are split into segments.  How long are the segments?
#define SEGMENT_PER_CM_ARC   (3)  // Arcs are split into segments.  How long are the segments?

// servo angles for pen control
#define PEN_UP_ANGLE         (90)
#define PEN_DOWN_ANGLE       (50)  // Some steppers don't like 0 degrees


#define NUM_TOOLS            (1)

#define MAX_FEEDRATE         (9000.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (100)
#define MAX_JERK             (5.0)
#define DEFAULT_FEEDRATE     (7000.0)
#define DEFAULT_ACCELERATION (2500)



#if DELTA_STYLE  == DELTA_STYLE_MARGINALLYCLEVER_V8
// physical measurements of the machine
#define CENTER_TO_SHOULDER       ( 3.770f)  // cm (f)
#define SHOULDER_TO_ELBOW        ( 5.000f)  // cm (Rf)
#define ELBOW_TO_WRIST           (16.500f)  // cm (Re)
#define EFFECTOR_TO_WRIST        ( 1.724f)  // cm (e)
#define CENTER_TO_FLOOR          (18.900f)  // cm
#define DEGREES_ABOVE_HORIZONTAL (24.000f)
#define NUM_AXIES            (3)
#define NUM_MOTORS           (3)
#define NUM_SERVOS           (0)
#endif


#if DELTA_STYLE  == DELTA_STYLE_JUDAH
// physical measurements of the machine
#define CENTER_TO_SHOULDER       (12.65682f)  // cm (f)
#define SHOULDER_TO_ELBOW        (20.320f)  // cm (Rf)
#define ELBOW_TO_WRIST           (55.820f)  // cm (Re)
#define EFFECTOR_TO_WRIST        ( 1.724f)  // cm (E)
#define CENTER_TO_FLOOR          (60.000f)  // cm
#define DEGREES_ABOVE_HORIZONTAL (24.000f)
#define NUM_AXIES            (4)
#define NUM_MOTORS           (4)
#define NUM_SERVOS           (0)
#endif


#endif  // #ifdef DELTA


#endif  // #ifndef ROBOT_TRADITIONALXY_H


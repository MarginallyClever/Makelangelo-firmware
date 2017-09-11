#ifndef ROBOT_DELTA_H
#define ROBOT_DELTA_H
//------------------------------------------------------------------------------
// Makelangelo - a mural drawing robot
// dan@marginallycelver.com 2013-12-26
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/Makelangelo for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == DELTA

#define MAKELANGELO_HARDWARE_VERSION 5  // If you have a makelangelo 5+

// known machine configurations
#define DELTA_STYLE_MARGINALLYCLEVER_V8  1
#define DELTA_STYLE_JUDAH                2

#define DELTA_STYLE DELTA_STYLE_MARGINALLYCLEVER_V8  // change this


#define STEP_DELAY           (50)  // delay between steps, in milliseconds, when doing fixed tasks like homing

#define MAX_ACCELERATION     (5000)
#define MIN_ACCELERATION     (100)
// servo angles for pen control
#define PEN_UP_ANGLE         (90)
#define PEN_DOWN_ANGLE       (50)  // Some steppers don't like 0 degrees


#define NUM_AXIES            (4)
#define NUM_MOTORS           (3)
#define NUM_SERVOS           (1)
#define NUM_TOOLS            (1)

#define MAX_FEEDRATE         (9000.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (100)
#define MAX_JERK             (5.0)
#define DEFAULT_FEEDRATE     (7000.0)
#define DEFAULT_ACCELERATION (2500)



#if DELTA_STYLE  == DELTA_STYLE_MARGINALLYCLEVER_V8
// physical measurements of the machine
#define CENTER_TO_SHOULDER       (3.77f)  // cm
#define SHOULDER_TO_ELBOW        (5.0f)  // cm
#define ELBOW_TO_WRIST           (16.5f)  // cm
#define EFFECTOR_TO_WRIST        (1.724f)  // cm
#define CENTER_TO_FLOOR          (18.9)  // cm
#define DEGREES_ABOVE_HORIZONTAL (24.0f)
#endif


#if DELTA_STYLE  == DELTA_STYLE_JUDAH
// physical measurements of the machine
#define CENTER_TO_SHOULDER       (12.65682f)  // cm (f)
#define SHOULDER_TO_ELBOW        (20.320f)  // cm (Rf)
#define ELBOW_TO_WRIST           (55.820f)  // cm (Re)
#define EFFECTOR_TO_WRIST        ( 1.724f)  // cm (E)
#define CENTER_TO_FLOOR          (60.000f)  // cm
#define DEGREES_ABOVE_HORIZONTAL (24.000f)
#endif


#endif  // #ifdef DELTA


#endif  // #ifndef ROBOT_TRADITIONALXY_H


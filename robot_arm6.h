#ifndef ROBOT_ARM6_H
#define ROBOT_ARM6_H
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == ARM6

#define MACHINE_STYLE_NAME           "ARM6"
#define MACHINE_HARDWARE_VERSION     5

#define STEP_DELAY           (50)  // delay between steps, in milliseconds, when doing fixed tasks like homing

#define MAX_ACCELERATION     (5000)
#define MIN_ACCELERATION     (100)

#define SUBDIVIDE_LINES
#define SEGMENT_PER_CM_LINE  (2)  // lines are split into segments.  How long are the segments?
#define SEGMENT_PER_CM_ARC   (3)  // Arcs are split into segments.  How long are the segments?

#define NUM_TOOLS            (1)
#define NUM_AXIES            (6)
#define NUM_MOTORS           (6)
#define NUM_SERVOS           (0)

#define MAX_FEEDRATE         (9000.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (100)
#define MAX_JERK             (5.0)
#define DEFAULT_FEEDRATE     (7000.0)
#define DEFAULT_ACCELERATION (2500)


#define MOTOR_MICROSTEPS_EXT      (1600)
#define MOTOR_MICROSTEPS_MID      (3200)
#define MOTOR_MICROSTEPS_INT      (200)
#define NEMA23_RATIO              (46.656)
#define NEMA17_RATIO              (15)
#define NEMA14_RATIO              (19)
#define NEMA08_RATIO              (90)
#define MOTOR_0_STEPS_PER_TURN    ((unsigned long)MOTOR_MICROSTEPS_EXT*NEMA23_RATIO)
#define MOTOR_1_STEPS_PER_TURN    ((unsigned long)MOTOR_MICROSTEPS_EXT*NEMA23_RATIO)
#define MOTOR_2_STEPS_PER_TURN    ((unsigned long)MOTOR_MICROSTEPS_MID*NEMA17_RATIO)
#define MOTOR_3_STEPS_PER_TURN    ((unsigned long)MOTOR_MICROSTEPS_MID*NEMA17_RATIO)
#define MOTOR_4_STEPS_PER_TURN    ((unsigned long)MOTOR_MICROSTEPS_INT*NEMA14_RATIO)
#define MOTOR_5_STEPS_PER_TURN    ((unsigned long)MOTOR_MICROSTEPS_INT*NEMA08_RATIO)


#endif  // #ifdef ARM6


#endif  // #ifndef ROBOT_ARM6_H


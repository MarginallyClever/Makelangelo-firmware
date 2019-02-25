#ifndef ROBOT_ARM6_H
#define ROBOT_ARM6_H
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == ARM6

#define MACHINE_STYLE_NAME           "ARM6"
//#define MACHINE_HARDWARE_VERSION     5  // red sixi 2017-2018
#define MACHINE_HARDWARE_VERSION     6  // yellow sixi 2019

#define STEP_DELAY           (1000)  // delay between steps, in milliseconds, when doing fixed tasks like homing

#define MAX_ACCELERATION     (5000)
#define MIN_ACCELERATION     (100)

//#define SUBDIVIDE_LINES
#define SEGMENT_PER_CM_LINE  (2)  // lines are split into segments.  How long are the segments?
#define SEGMENT_PER_CM_ARC   (3)  // Arcs are split into segments.  How long are the segments?

#define NUM_AXIES            (7)
#define NUM_MOTORS           (6)
#define NUM_SERVOS           (1)
#define NUM_TOOLS            (1)

#define MAX_FEEDRATE         (400.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (0)
#define MAX_JERK             (5.0)
#define DEFAULT_FEEDRATE     (100.0)
#define DEFAULT_ACCELERATION (500)

#define MAX_SEGMENTS 16  // override the default to save RAM

//#define HAS_LCD
#define HAS_SD

#if MACHINE_HARDWARE_VERSION==6

#define MOTOR_MICROSTEPS_EXT      ((1.0*200.0))  // motor full steps * microstepping setting
#define MOTOR_MICROSTEPS_MID      ((1.0*400.0))  // motor full steps * microstepping setting

#define NEMA17_CYCLOID_GEARBOX_RATIO (40.0)
#define ELBOW_DOWNGEAR_RATIO         (30.0/14.0)
#define NEMA17_RATIO                 (NEMA17_CYCLOID_GEARBOX_RATIO*ELBOW_DOWNGEAR_RATIO)

#define MOTOR_0_STEPS_PER_TURN    (MOTOR_MICROSTEPS_MID*NEMA17_RATIO)
#define MOTOR_1_STEPS_PER_TURN    (MOTOR_MICROSTEPS_MID*NEMA17_RATIO)
#define MOTOR_2_STEPS_PER_TURN    (MOTOR_MICROSTEPS_EXT*NEMA17_RATIO)

#define MOTOR_3_STEPS_PER_TURN    (MOTOR_MICROSTEPS_MID*NEMA17_RATIO)
#define MOTOR_4_STEPS_PER_TURN    (MOTOR_MICROSTEPS_EXT*NEMA17_RATIO)
#define MOTOR_5_STEPS_PER_TURN    (MOTOR_MICROSTEPS_EXT*NEMA17_RATIO)


// DIMENSIONS

#define ADJUST_WRIST_ELBOW_ANGLE      (14.036243)
#define ADJUST_SHOULDER_ELBOW_ANGLE   (11.309932)
#define ADJUST_ULNA_ELBOW_ANGLE       (26.56505117707799)

// Relative offset from joint N to joint N+1.
// shoulder pan
#define OFFSET_0X    0
#define OFFSET_0Y    0
#define OFFSET_0Z    0
// shoulder tilt
#define OFFSET_1X    0
#define OFFSET_1Y    0
#define OFFSET_1Z    258
// elbow tilt
#define OFFSET_2X    0
#define OFFSET_2Y    50
#define OFFSET_2Z    250
// elbow pan
#define OFFSET_3X    0
#define OFFSET_3Y    -50
#define OFFSET_3Z    100
// wrist
#define OFFSET_4X    0
#define OFFSET_4Y    0
#define OFFSET_4Z    100
// wrist to wrist face
#define OFFSET_5X    0
#define OFFSET_5Y    0
#define OFFSET_5Z    50

// angle (degrees) when homing switches report in
#define MIN_ANGLE_0 0
#define MIN_ANGLE_1 -90
#define MIN_ANGLE_2 -11.1
#define MIN_ANGLE_3 0
#define MIN_ANGLE_4 -90
#define MIN_ANGLE_5 0

#define MAX_ANGLE_0 180
#define MAX_ANGLE_1 90
#define MAX_ANGLE_2 (180-11.1)
#define MAX_ANGLE_3 180
#define MAX_ANGLE_4 90
#define MAX_ANGLE_5 180

#define HOME_ANGLE_0 MIN_ANGLE_0
#define HOME_ANGLE_1 MIN_ANGLE_1
#define HOME_ANGLE_2 MIN_ANGLE_2
#define HOME_ANGLE_3 MIN_ANGLE_3
#define HOME_ANGLE_4 MIN_ANGLE_4
#define HOME_ANGLE_5 MIN_ANGLE_5

#define HOME_DIR_0 HIGH
#define HOME_DIR_1 HIGH
#define HOME_DIR_2 LOW
#define HOME_DIR_3 HIGH
#define HOME_DIR_4 LOW
#define HOME_DIR_5 HIGH

#endif  // MACHINE_HARDWARE_VERSION==6
#if MACHINE_HARDWARE_VERSION==5

#define MOTOR_MICROSTEPS_EXT      (1600)  // motor full steps * microstepping setting
#define MOTOR_MICROSTEPS_MID      (3200)  // motor full steps * microstepping setting
#define MOTOR_MICROSTEPS_INT      ( 200)  // motor full steps * microstepping setting
#define NEMA23_RATIO              (46.656)
#define NEMA17_RATIO              (15)
#define NEMA14_RATIO              (19.19)
#define NEMA08_RATIO              (90.25)
#define MOTOR_0_STEPS_PER_TURN    ((unsigned long)MOTOR_MICROSTEPS_EXT*NEMA23_RATIO)
#define MOTOR_1_STEPS_PER_TURN    ((unsigned long)MOTOR_MICROSTEPS_EXT*NEMA23_RATIO)
#define MOTOR_2_STEPS_PER_TURN    ((unsigned long)MOTOR_MICROSTEPS_MID*NEMA17_RATIO)
#define MOTOR_3_STEPS_PER_TURN    ((unsigned long)MOTOR_MICROSTEPS_MID*NEMA17_RATIO)
#define MOTOR_4_STEPS_PER_TURN    ((unsigned long)MOTOR_MICROSTEPS_INT*NEMA14_RATIO)
#define MOTOR_5_STEPS_PER_TURN    ((unsigned long)MOTOR_MICROSTEPS_INT*NEMA08_RATIO)


// DIMENSIONS

#define ADJUST_WRIST_ELBOW_ANGLE      (14.036243)
#define ADJUST_SHOULDER_ELBOW_ANGLE   (11.309932)
#define ADJUST_ULNA_ELBOW_ANGLE       (26.56505117707799)

// Relative offset from joint N to joint N+1.
// shoulder pan
#define OFFSET_0X    0
#define OFFSET_0Y    0
#define OFFSET_0Z    0
// shoulder tilt
#define OFFSET_1X    0
#define OFFSET_1Y    0
#define OFFSET_1Z    258
// elbow tilt
#define OFFSET_2X    0
#define OFFSET_2Y    50
#define OFFSET_2Z    250
// elbow pan
#define OFFSET_3X    0
#define OFFSET_3Y    -50
#define OFFSET_3Z    100
// wrist
#define OFFSET_4X    0
#define OFFSET_4Y    0
#define OFFSET_4Z    100
// wrist to wrist face
#define OFFSET_5X    0
#define OFFSET_5Y    0
#define OFFSET_5Z    50

// angle (degrees) when homing switches report in
#define MIN_ANGLE_0 0
#define MIN_ANGLE_1 -90
#define MIN_ANGLE_2 -11.1
#define MIN_ANGLE_3 0
#define MIN_ANGLE_4 -90
#define MIN_ANGLE_5 0

#define MAX_ANGLE_0 180
#define MAX_ANGLE_1 90
#define MAX_ANGLE_2 (180-11.1)
#define MAX_ANGLE_3 180
#define MAX_ANGLE_4 90
#define MAX_ANGLE_5 180

#define HOME_ANGLE_0 MIN_ANGLE_0
#define HOME_ANGLE_1 MIN_ANGLE_1
#define HOME_ANGLE_2 MIN_ANGLE_2
#define HOME_ANGLE_3 MIN_ANGLE_3
#define HOME_ANGLE_4 MIN_ANGLE_4
#define HOME_ANGLE_5 MIN_ANGLE_5

#define HOME_DIR_0 HIGH
#define HOME_DIR_1 HIGH
#define HOME_DIR_2 LOW
#define HOME_DIR_3 HIGH
#define HOME_DIR_4 LOW
#define HOME_DIR_5 HIGH

#endif  // MACHINE_HARDWARE_VERSION==5

#endif  // #ifdef ARM6


#endif  // #ifndef ROBOT_ARM6_H

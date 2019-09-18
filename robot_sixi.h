#ifndef ROBOT_SIXI_H
#define ROBOT_SIXI_H
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == SIXI

#define MACHINE_STYLE_NAME           "SIXI"
#define MACHINE_HARDWARE_VERSION     6  // yellow sixi 2019

#define STEP_DELAY           (50)  // delay between steps, in milliseconds, when doing fixed tasks like homing

// plan long moves as a set of submoves to increase accuracy.  Uncomment to turn this off.
#define SUBDIVIDE_LINES
// what is the maximum length of a subdivided line?
#define SEGMENT_MAX_LENGTH_MM  (5)

#define NUM_AXIES            (7)
#define NUM_MOTORS           (6)
#define NUM_SERVOS           (1)
#define NUM_TOOLS            (1)
#define NUM_SENSORS          (6)

#define MAX_FEEDRATE         (80.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (0)
#define DEFAULT_FEEDRATE     (45.0)

#define MAX_ACCELERATION     (2000)
#define MIN_ACCELERATION     (0)
#define DEFAULT_ACCELERATION (10)

#define MAX_JERK             (5.0)

#define MAX_SEGMENTS         (16)  // override the default to save RAM
#define DEGREES_PER_STEP     (1.8)
#define MICROSTEPS           (1.0)

//#define HAS_LCD
#define HAS_SD
#define MOTHERBOARD BOARD_SIXI_MEGA  // sixi only supports one motherboard right now


#define MOTOR_STEPS_PER_TURN          (200.0)  // motor full steps * microstepping setting

#define NEMA17_CYCLOID_GEARBOX_RATIO        (20.0)
#define NEMA23_CYCLOID_GEARBOX_RATIO_ELBOW  (35.0)
#define NEMA23_CYCLOID_GEARBOX_RATIO_ANCHOR (30.0)
#define NEMA24_CYCLOID_GEARBOX_RATIO        (40.0)

#define DM322T_MICROSTEP              (2.0)

#define ELBOW_DOWNGEAR_RATIO          (30.0/20.0)
#define NEMA17_RATIO                  (DM322T_MICROSTEP*NEMA17_CYCLOID_GEARBOX_RATIO*ELBOW_DOWNGEAR_RATIO)
#define NEMA23_RATIO_ELBOW            (NEMA23_CYCLOID_GEARBOX_RATIO_ELBOW)
#define NEMA23_RATIO_ANCHOR           (NEMA23_CYCLOID_GEARBOX_RATIO_ANCHOR)
#define NEMA24_RATIO                  (NEMA24_CYCLOID_GEARBOX_RATIO)

// Motors are numbered 0 (base) to 5 (hand)
#define MOTOR_0_STEPS_PER_TURN    (MOTOR_STEPS_PER_TURN*NEMA23_RATIO_ANCHOR)  // anchor
#define MOTOR_1_STEPS_PER_TURN    (MOTOR_STEPS_PER_TURN*NEMA24_RATIO)  // shoulder
#define MOTOR_2_STEPS_PER_TURN    (MOTOR_STEPS_PER_TURN*NEMA23_RATIO_ELBOW)  // elbow
#define MOTOR_3_STEPS_PER_TURN    (MOTOR_STEPS_PER_TURN*NEMA17_RATIO)  // ulna
#define MOTOR_4_STEPS_PER_TURN    (MOTOR_STEPS_PER_TURN*NEMA17_RATIO)  // wrist
#define MOTOR_5_STEPS_PER_TURN    (MOTOR_STEPS_PER_TURN*NEMA17_RATIO)  // hand

#define DEGREES_PER_STEP_0 (360.0/MOTOR_0_STEPS_PER_TURN)
#define DEGREES_PER_STEP_1 (360.0/MOTOR_1_STEPS_PER_TURN)
#define DEGREES_PER_STEP_2 (360.0/MOTOR_2_STEPS_PER_TURN)
#define DEGREES_PER_STEP_3 (360.0/MOTOR_3_STEPS_PER_TURN)
#define DEGREES_PER_STEP_4 (360.0/MOTOR_4_STEPS_PER_TURN)
#define DEGREES_PER_STEP_5 (360.0/MOTOR_5_STEPS_PER_TURN)

#define POSITION_EPSILON (1.9) // degrees

// step signal start
#define START0 LOW
#define START1 LOW
#define START2 LOW
#define START3 HIGH
#define START4 HIGH
#define START5 HIGH

// step signal end
#define END0 HIGH
#define END1 HIGH
#define END2 HIGH
#define END3 LOW
#define END4 LOW
#define END5 LOW

extern float sensorAngles[NUM_SENSORS];

extern void sensorUpdate();

#endif  // #ifdef SIXI


#endif  // #ifndef ROBOT_SIXI_H

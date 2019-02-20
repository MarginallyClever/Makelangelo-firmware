#ifndef BOARD_SANGUINOLULU_H
#define BOARD_SANGUINOLULU_H
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------


#if MOTHERBOARD == BOARD_SANGUINOLULU 
#define MAX_MOTORS                 (4)
//x
#define MOTOR_0_DIR_PIN           (21)
#define MOTOR_0_STEP_PIN          (15)
#define MOTOR_0_ENABLE_PIN        (14)
#define MOTOR_0_LIMIT_SWITCH_PIN  (18)
//y
#define MOTOR_1_DIR_PIN           (23)
#define MOTOR_1_STEP_PIN          (22)
#define MOTOR_1_ENABLE_PIN        (14)
#define MOTOR_1_LIMIT_SWITCH_PIN  (19)
//z
#define MOTOR_2_DIR_PIN           (2)
#define MOTOR_2_STEP_PIN          (3)
#define MOTOR_2_ENABLE_PIN        (14)
#define MOTOR_2_LIMIT_SWITCH_PIN  (20)
//e
#define MOTOR_3_DIR_PIN           (0)
#define MOTOR_3_STEP_PIN          (1)
#define MOTOR_3_ENABLE_PIN        (14)
// the sanguinolulu doesn't have a limit switch pin for E.
// I've assigned it to pin 24, the e-thermistor pin.  I have no idea if this will work.
#define MOTOR_3_LIMIT_SWITCH_PIN  (24)

#define MAX_BOARD_SERVOS           (1)
#define SERVO0_PIN                (12)

#undef HAS_LCD
#undef HAS_SD

#endif


#endif  // BOARD_SANGUINOLULU_H

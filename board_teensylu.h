#ifndef BOARD_TEENSYLU_H
#define BOARD_TEENSYLU_H
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------


#if MOTHERBOARD == BOARD_TEENSYLU
#define MAX_MOTORS                 (2)

#define MOTOR_0_DIR_PIN           (29)
#define MOTOR_0_STEP_PIN          (28)
#define MOTOR_0_ENABLE_PIN        (19)
#define MOTOR_0_LIMIT_SWITCH_PIN  (26)

#define MOTOR_1_DIR_PIN           (31)
#define MOTOR_1_STEP_PIN          (30)
#define MOTOR_1_ENABLE_PIN        (18)
#define MOTOR_1_LIMIT_SWITCH_PIN  (27)

#define NUM_BOARD_SERVOS          (1)
#define SERVO0_PIN                (24)

#endif


#endif  // BOARD_TEENSYLU_H


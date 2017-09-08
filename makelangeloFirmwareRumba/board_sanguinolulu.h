#ifndef BOARD_RAMPS_H
#define BOARD_RAMPS_H
//------------------------------------------------------------------------------
// Makelangelo - a mural drawing robot
// dan@marginallycelver.com 2013-12-26
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/Makelangelo for more information.
//------------------------------------------------------------------------------


#if MOTHERBOARD == BOARD_SANGUINOLULU 
#define MAX_MOTORS                 (2)

#define MOTOR_0_DIR_PIN           (21)
#define MOTOR_0_STEP_PIN          (15)
#define MOTOR_0_ENABLE_PIN        (14)
#define MOTOR_0_LIMIT_SWITCH_PIN  (18)

#define MOTOR_1_DIR_PIN           (23)
#define MOTOR_1_STEP_PIN          (22)
#define MOTOR_1_ENABLE_PIN        (14)
#define MOTOR_1_LIMIT_SWITCH_PIN  (19)

// TODO: if ZARPLOTTER & SANGUINOLULU throw a compile error, not enough motors.

#define MAX_BOARD_SERVOS           (1)
#define SERVO0_PIN                (12)
#endif


#endif  // BOARD_RAMPS_H


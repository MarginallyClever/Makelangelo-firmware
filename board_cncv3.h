#pragma once
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

// cnc shield pins: https://blog.protoneer.co.nz/arduino-cnc-shield/arduino-cnc-shield-scematics-v3-xx/

#if MOTHERBOARD == BOARD_CNCV3 

// wrong board type set
#ifndef __AVR_ATmega2560__
  #error "Oops!  Make sure you have 'Arduino Mega or Mega 2560' selected from the 'Tools -> Boards' menu."
#endif


#define MAX_SEGMENTS              (1)


// actual limit is 4 but I only have the pins for the first two motors.
// TODO add more pin definitions
#define MAX_MOTORS                (3)

#define MOTOR_0_LETTER            'X'
#define MOTOR_0_DIR_PIN           (5)
#define MOTOR_0_STEP_PIN          (2)
#define MOTOR_0_ENABLE_PIN        (8)
#define MOTOR_0_LIMIT_SWITCH_PIN  (9)   /* X min */

#define MOTOR_1_LETTER            'Y'
#define MOTOR_1_DIR_PIN           (6)
#define MOTOR_1_STEP_PIN          (3)
#define MOTOR_1_ENABLE_PIN        (8)
#define MOTOR_1_LIMIT_SWITCH_PIN  (10)  /* Y min */

#define MOTOR_2_LETTER            'Z'
#define MOTOR_2_DIR_PIN           (7)
#define MOTOR_2_STEP_PIN          (4)
#define MOTOR_2_ENABLE_PIN        (8)
#define MOTOR_2_LIMIT_SWITCH_PIN  (11)  /* Z min */

#define MAX_BOARD_SERVOS          (1)
#define SERVO0_PIN                (23)   /* Servo 1 */

#define LIMIT_SWITCH_PIN_LEFT     (MOTOR_0_LIMIT_SWITCH_PIN)
#define LIMIT_SWITCH_PIN_RIGHT    (MOTOR_1_LIMIT_SWITCH_PIN)

#define CLOCK_FREQ                (16000000L)

#define _useTimer2
#undef _useTimer3

#undef HAS_SD  
#undef HAS_LCD

#endif // MOTHERBOARD == BOARD_CNCV3

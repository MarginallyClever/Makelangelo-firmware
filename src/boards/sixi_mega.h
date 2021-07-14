#pragma once
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#if MOTHERBOARD == BOARD_SIXI_MEGA

// wrong board type set
#  ifndef __AVR_ATmega2560__
#    error "Oops!  Make sure you have 'Arduino Mega 2560' selected from the 'Tools -> Boards' menu."
#  endif

#  define BAUD (57600)  // How fast is the Arduino talking?

#  undef HAS_LCD

#  ifdef HAS_SD
#    undef HAS_SD
#  endif

#  define MAX_MOTORS (6)

#  define MOTOR_0_LETTER     'X'
#  define MOTOR_0_DIR_PIN    46
#  define MOTOR_0_STEP_PIN   45
#  define MOTOR_0_ENABLE_PIN 47

#  define MOTOR_1_LETTER     'Y'
#  define MOTOR_1_DIR_PIN    43
#  define MOTOR_1_STEP_PIN   42
#  define MOTOR_1_ENABLE_PIN 44

#  define MOTOR_2_LETTER     'Z'
#  define MOTOR_2_DIR_PIN    40
#  define MOTOR_2_STEP_PIN   39
#  define MOTOR_2_ENABLE_PIN 41

#  define MOTOR_3_LETTER     'U'
#  define MOTOR_3_DIR_PIN    37
#  define MOTOR_3_STEP_PIN   36
#  define MOTOR_3_ENABLE_PIN 38

#  define MOTOR_4_LETTER     'V'
#  define MOTOR_4_DIR_PIN    34
#  define MOTOR_4_STEP_PIN   33
#  define MOTOR_4_ENABLE_PIN 35

#  define MOTOR_5_LETTER     'W'
#  define MOTOR_5_DIR_PIN    31
#  define MOTOR_5_STEP_PIN   30
#  define MOTOR_5_ENABLE_PIN 32

// These are dummy placeholders because it's easier than rewriting the rest of the code.
#  define MOTOR_0_LIMIT_SWITCH_PIN 9  //(37)  // x-
#  define MOTOR_1_LIMIT_SWITCH_PIN 9  //(36)  // x+
#  define MOTOR_2_LIMIT_SWITCH_PIN 9  //(35)  // y-
#  define MOTOR_3_LIMIT_SWITCH_PIN 9  //(34)  // y+
#  define MOTOR_4_LIMIT_SWITCH_PIN 9  //(33)  // z-
#  define MOTOR_5_LIMIT_SWITCH_PIN 9  //(32)  // z+

#  define MAX_BOARD_SERVOS (1)
#  define SERVO0_PIN       (13)

// pins
#  define PIN_SENSOR_CSEL_0 8
#  define PIN_SENSOR_CLK_0  9
#  define PIN_SENSOR_MOSI_0 10
#  define PIN_SENSOR_MISO_0 11

#  define PIN_SENSOR_CSEL_1 2
#  define PIN_SENSOR_CLK_1  3
#  define PIN_SENSOR_MOSI_1 4
#  define PIN_SENSOR_MISO_1 5

#  define PIN_SENSOR_CSEL_2 17
#  define PIN_SENSOR_CLK_2  16
#  define PIN_SENSOR_MOSI_2 15
#  define PIN_SENSOR_MISO_2 14

#  define PIN_SENSOR_CSEL_3 21
#  define PIN_SENSOR_CLK_3  20
#  define PIN_SENSOR_MOSI_3 19
#  define PIN_SENSOR_MISO_3 18

// every unit after the first
#  define PIN_SENSOR_CSEL_4 22
#  define PIN_SENSOR_CLK_4  24
#  define PIN_SENSOR_MOSI_4 26
#  define PIN_SENSOR_MISO_4 28

#  define PIN_SENSOR_CSEL_5 29
#  define PIN_SENSOR_CLK_5  27
#  define PIN_SENSOR_MOSI_5 25
#  define PIN_SENSOR_MISO_5 23

#  ifdef HAS_GRIPPER
#    define TEST_GRIPPER_PIN (12)
#  endif

#include "AVR/hal.h"

#endif  // MOTHERBOARD == BOARD_SIXI_MEGA

#pragma once
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

// https://www.instructables.com/id/Programming-the-WeMos-Using-Arduino-SoftwareIDE/

// cnc shield pins: https://blog.protoneer.co.nz/arduino-cnc-shield/arduino-cnc-shield-scematics-v3-xx/
// WEMOS D1 R2 pins: https://protosupplies.com/wp-content/uploads/2018/07/Wemos-D1-Pin-Differences.pdf.jpg
// WEMOS D1 R2 board type should be set to "LOLIN(WEMOS) R1 D2 & mini"

#if MOTHERBOARD == BOARD_WEMOS

// wrong board type set
#  ifndef ESP8266
#    error "Oops!  Make sure you have 'Wemos D1 R1' selected from the 'Tools -> Boards' menu."
#  endif

#define HAL_STEP_TIMER_ISR void itr

// actual limit is 4 but I only have the pins for the first two motors.
// TODO add more pin definitions
#  define MAX_MOTORS (2)

#  define MOTOR_0_LETTER           'X'
#  define MOTOR_0_DIR_PIN          (3)
#  define MOTOR_0_STEP_PIN         (0)
#  define MOTOR_0_ENABLE_PIN       (6)
#  define MOTOR_0_LIMIT_SWITCH_PIN (7) /* X min */

#  define MOTOR_1_LETTER           'Y'
#  define MOTOR_1_DIR_PIN          (4)
#  define MOTOR_1_STEP_PIN         (1)
#  define MOTOR_1_ENABLE_PIN       (6)
#  define MOTOR_1_LIMIT_SWITCH_PIN (8) /* Y min */

#  define MAX_BOARD_SERVOS (1)
#  define SERVO0_PIN       (23) /* Servo 1 */

#  define LIMIT_SWITCH_PIN_LEFT  (MOTOR_0_LIMIT_SWITCH_PIN)
#  define LIMIT_SWITCH_PIN_RIGHT (MOTOR_1_LIMIT_SWITCH_PIN)

#  define HAS_WIFI
#  define WIFI_SSID_NAME ""  // WiFi AP SSID Name - put this in your local_config.h
#  define WIFI_SSID_PASS ""  // WiFi AP SSID Password - put this in your local_config.h

#  undef HAS_SD
#  undef HAS_LCD

#include "HAL_esp8266.h"

#endif  // MOTHERBOARD == BOARD_WEMOS

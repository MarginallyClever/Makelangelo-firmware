#pragma once
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

// https://www.instructables.com/id/Programming-the-ESP32-Using-Arduino-SoftwareIDE/

// ESP32 D1 R2 board type should be set to "LOLIN(ESP32) R1 D2 & mini"
// or WEMOS D1 ESP32 MINI
 
#if MOTHERBOARD == BOARD_ESP32 

// wrong board type set
#ifndef ESP32
  #error "Oops!  Make sure you have 'ESP32 D1 R1' selected from the 'Tools -> Boards' menu."
#endif

#define MAX_MOTORS                (2)

#define MOTOR_0_LETTER            'X'
#define MOTOR_0_DIR_PIN           (15)
#define MOTOR_0_STEP_PIN          (13)
#define MOTOR_0_ENABLE_PIN        (12)
#define MOTOR_0_LIMIT_SWITCH_PIN  ()   /* X min */

#define MOTOR_1_LETTER            'Y'
#define MOTOR_1_DIR_PIN           (0)
#define MOTOR_1_STEP_PIN          (4)
#define MOTOR_1_ENABLE_PIN        (5)
#define MOTOR_1_LIMIT_SWITCH_PIN  ()  /* Y min */

#define CS_PIN_0 9
#define DIAG1_0  14

#define CS_PIN_1 10
#define DIAG1_1  16

#define MAX_BOARD_SERVOS          (1)
#define SERVO0_PIN                (23)   /* Servo 1 */

#define LIMIT_SWITCH_PIN_LEFT     (MOTOR_0_LIMIT_SWITCH_PIN)
#define LIMIT_SWITCH_PIN_RIGHT    (MOTOR_1_LIMIT_SWITCH_PIN)

#define HAS_WIFI
#define WIFI_SSID_NAME ""  // WiFi AP SSID Name - define this in local_config.h
#define WIFI_SSID_PASS ""  // WiFi AP SSID Password - define this in local_config.h

#define CLOCK_FREQ            (80000000L)

#undef HAS_SD  
#undef HAS_LCD

#endif // MOTHERBOARD == BOARD_ESP32 

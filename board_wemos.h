#ifndef BOARD_WEMOS_H
#define BOARD_WEMOS_H
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
#ifndef ESP8266
  #error "Oops!  Make sure you have 'Wemos D1 R1' selected from the 'Tools -> Boards' menu."
#endif

// actual limit is 4 but I only have the pins for the first two motors.
// TODO add more pin definitions
#define MAX_MOTORS                (2)

#define MOTOR_0_DIR_PIN           (3)
#define MOTOR_0_STEP_PIN          (0)
#define MOTOR_0_ENABLE_PIN        (6)
#define MOTOR_0_LIMIT_SWITCH_PIN  (7)   /* X min */

#define MOTOR_1_DIR_PIN           (4)
#define MOTOR_1_STEP_PIN          (1)
#define MOTOR_1_ENABLE_PIN        (6)
#define MOTOR_1_LIMIT_SWITCH_PIN  (8)  /* Y min */

#define MAX_BOARD_SERVOS          (1)
#define SERVO0_PIN                (23)   /* Servo 1 */

#define LIMIT_SWITCH_PIN_LEFT     (MOTOR_0_LIMIT_SWITCH_PIN)
#define LIMIT_SWITCH_PIN_RIGHT    (MOTOR_1_LIMIT_SWITCH_PIN)

#define HAS_WIFI
#define WIFI_SSID_NAME "CSIS VAN #1"    // WiFi AP SSID Name
#define WIFI_SSID_PASS "TDPJYGZEDH123"  // WiFi AP SSID Password

#define CLOCK_FREQ            (80000000L)

#undef HAS_SD  
#undef HAS_LCD

#endif // MOTHERBOARD == BOARD_WEMOS 

#endif  // BOARD_WEMOS_H

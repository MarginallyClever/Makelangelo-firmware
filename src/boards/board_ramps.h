#pragma once
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------


#if MOTHERBOARD == BOARD_RAMPS 
// wrong board type set
#ifndef __AVR_ATmega2560__
  #error "Oops!  Make sure you have 'Arduino Mega 2560' selected from the 'Tools -> Boards' menu."
#endif

#define MAX_MOTORS                 (5)

#define MOTOR_0_LETTER            'X'
#define MOTOR_0_DIR_PIN           (55)
#define MOTOR_0_STEP_PIN          (54)
#define MOTOR_0_ENABLE_PIN        (38)
#define MOTOR_0_LIMIT_SWITCH_PIN  (3)   /* X min */

#define MOTOR_1_LETTER            'Y'
#define MOTOR_1_DIR_PIN           (61)
#define MOTOR_1_STEP_PIN          (60)
#define MOTOR_1_ENABLE_PIN        (56)
#define MOTOR_1_LIMIT_SWITCH_PIN  (14)  /* Y min */

#define MOTOR_2_LETTER            'Z'
#define MOTOR_2_DIR_PIN           (48)
#define MOTOR_2_STEP_PIN          (46)
#define MOTOR_2_ENABLE_PIN        (62)
#define MOTOR_2_LIMIT_SWITCH_PIN  (18)  /* Z Min */

#define MOTOR_3_LETTER            'U'
#define MOTOR_3_DIR_PIN           (28)
#define MOTOR_3_STEP_PIN          (26)
#define MOTOR_3_ENABLE_PIN        (24)
#define MOTOR_3_LIMIT_SWITCH_PIN  (2)   /* X Max */

#define MOTOR_4_LETTER            'V'
#define MOTOR_4_DIR_PIN           (34)
#define MOTOR_4_STEP_PIN          (36)
#define MOTOR_4_ENABLE_PIN        (30)
#define MOTOR_4_LIMIT_SWITCH_PIN  (15)  /* Y Max */

#define MAX_BOARD_SERVOS          (4)
#define SERVO0_PIN                (11)   /* Servo 1 */
#define SERVO1_PIN                (6)
#define SERVO2_PIN                (5)
#define SERVO3_PIN                (4)

#define LIMIT_SWITCH_PIN_LEFT     (MOTOR_0_LIMIT_SWITCH_PIN)
#define LIMIT_SWITCH_PIN_RIGHT    (MOTOR_1_LIMIT_SWITCH_PIN)

// LCD pins
#if LCD_TYPE == LCD_IS_128X64
// 128x64 full graphics controller
#define BEEPER             37

#define LCD_PINS_RS        19
#define LCD_PINS_ENABLE    42
#define LCD_PINS_D4        18
// these alternate pins might help - see https://www.marginallyclever.com/forums/topic/using-reprap-kit-from-amazon-ramps-1-6-lcd-display/#post-22131
//#define LCD_PINS_RS        16
//#define LCD_PINS_ENABLE    17
//#define LCD_PINS_D4        23

#define LCD_PINS_D5        25
#define LCD_PINS_D6        27
#define LCD_PINS_D7        29

// Encoder rotation values
#define BTN_EN1            31
#define BTN_EN2            33
#define BTN_ENC            35

// SD card settings
#define SDPOWER            -1
#define SDSS               53
#define SDCARDDETECT       49

#define KILL_PIN           41
#endif // LCD_IS_128X64

#if LCD_TYPE == LCD_IS_SMART
// Smart controller settings
#define BEEPER             37
#define LCD_PINS_RS        16
#define LCD_PINS_ENABLE    17
#define LCD_PINS_D4        23
#define LCD_PINS_D5        25
#define LCD_PINS_D6        27
#define LCD_PINS_D7        29

// Encoder rotation values
#define BTN_EN1            31
#define BTN_EN2            33
#define BTN_ENC            35

// SD card settings
#define SDPOWER            -1
#define SDSS               53
#define SDCARDDETECT       49

#define KILL_PIN           41
#endif  // LCD_IS_SMART

#endif

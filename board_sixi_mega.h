#ifndef BOARD_SIXI_MEGA_H
#define BOARD_SIXI_MEGA_H
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------


#if MOTHERBOARD == BOARD_SIXI_MEGA 
// wrong board type set
#ifndef __AVR_ATmega2560__
  #error "Oops!  Make sure you have 'Arduino Mega 2560' selected from the 'Tools -> Boards' menu."
#endif

#define MAX_MOTORS                 (6)

#define MOTOR_0_DIR_PIN           48//(16)
#define MOTOR_0_STEP_PIN          50//(17)
#define MOTOR_0_ENABLE_PIN        52//(48)
#define MOTOR_0_LIMIT_SWITCH_PIN  51//(37)  // x-

#define MOTOR_1_DIR_PIN           43//(47)
#define MOTOR_1_STEP_PIN          45//(54)
#define MOTOR_1_ENABLE_PIN        47//(55)
#define MOTOR_1_LIMIT_SWITCH_PIN  51//(36)  // x+

#define MOTOR_2_DIR_PIN           36//(56)
#define MOTOR_2_STEP_PIN          38//(57)
#define MOTOR_2_ENABLE_PIN        40//(62)
#define MOTOR_2_LIMIT_SWITCH_PIN  51//(35)  // y-

#define MOTOR_3_DIR_PIN           31//(22)
#define MOTOR_3_STEP_PIN          33//(23)
#define MOTOR_3_ENABLE_PIN        53//(24)
#define MOTOR_3_LIMIT_SWITCH_PIN  51//(34)  // y+

#define MOTOR_4_DIR_PIN           26//(25)
#define MOTOR_4_STEP_PIN          28//(26)
#define MOTOR_4_ENABLE_PIN        53//(27)
#define MOTOR_4_LIMIT_SWITCH_PIN  51//(33)  // z-

#define MOTOR_5_DIR_PIN           23//(28)
#define MOTOR_5_STEP_PIN          25//(29)
#define MOTOR_5_ENABLE_PIN        53//(39)
#define MOTOR_5_LIMIT_SWITCH_PIN  51//(32)  // z+

#define MAX_BOARD_SERVOS          (1)
#define SERVO0_PIN                (5)

#define LIMIT_SWITCH_PIN_LEFT     (MOTOR_0_LIMIT_SWITCH_PIN)
#define LIMIT_SWITCH_PIN_RIGHT    (MOTOR_1_LIMIT_SWITCH_PIN)

#undef HAS_LCD

// LCD pins
#ifdef LCD_IS_128X64
#define BEEPER             44
#define LCD_PINS_RS        19
#define LCD_PINS_ENABLE    42
#define LCD_PINS_D4        18
#define LCD_PINS_D5        38
#define LCD_PINS_D6        41
#define LCD_PINS_D7        40

// Encoder rotation values
#define BTN_EN1            11
#define BTN_EN2            12
#define BTN_ENC            43

// SD card settings
#define SDPOWER            -1
#define SDSS               53
#define SDCARDDETECT       49
#endif  // LCD_IS_128x64

#ifdef LCD_IS_SMART
#define BEEPER             44
#define LCD_PINS_RS        19
#define LCD_PINS_ENABLE    42
#define LCD_PINS_D4        18
#define LCD_PINS_D5        38
#define LCD_PINS_D6        41
#define LCD_PINS_D7        40

// Encoder rotation values
#define BTN_EN1            11
#define BTN_EN2            12
#define BTN_ENC            43

// SD card settings
#define SDPOWER            -1
#define SDSS               53
#define SDCARDDETECT       49
#endif  // LCD_IS_SMART

#endif  // MOTHERBOARD == BOARD_SIXI_MEGA 


#endif  // BOARD_SIXI_MEGA_H

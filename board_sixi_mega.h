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

#define MOTOR_0_DIR_PIN           22//(16)
#define MOTOR_0_STEP_PIN          24//(17)
#define MOTOR_0_ENABLE_PIN        26//(48)
#define MOTOR_0_LIMIT_SWITCH_PIN  9//(37)  // x-

#define MOTOR_1_DIR_PIN           28//(47)
#define MOTOR_1_STEP_PIN          30//(54)
#define MOTOR_1_ENABLE_PIN        32//(55)
#define MOTOR_1_LIMIT_SWITCH_PIN  9//(36)  // x+

#define MOTOR_2_DIR_PIN           34//(56)
#define MOTOR_2_STEP_PIN          36//(57)
#define MOTOR_2_ENABLE_PIN        38//(62)
#define MOTOR_2_LIMIT_SWITCH_PIN  9//(35)  // y-

#define MOTOR_3_DIR_PIN           40//(22)
#define MOTOR_3_STEP_PIN          42//(23)
#define MOTOR_3_ENABLE_PIN        41//(24
)#define MOTOR_3_LIMIT_SWITCH_PIN  9//(34)  // y+

#define MOTOR_4_DIR_PIN           44//(25)
#define MOTOR_4_STEP_PIN          46//(26)
#define MOTOR_4_ENABLE_PIN        45//(27)
#define MOTOR_4_LIMIT_SWITCH_PIN  9//(33)  // z-

#define MOTOR_5_DIR_PIN           48//(28)
#define MOTOR_5_STEP_PIN          50//(29)
#define MOTOR_5_ENABLE_PIN        49//(39)
#define MOTOR_5_LIMIT_SWITCH_PIN  9//(32)  // z+

#define MAX_BOARD_SERVOS          (1)
#define SERVO0_PIN                (5)

#undef HAS_LCD
#undef HAS_SD

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

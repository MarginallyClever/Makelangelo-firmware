#ifndef BOARD_RUMBA_H
#define BOARD_RUMBA_H
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------


#if MOTHERBOARD == BOARD_RUMBA 
// wrong board type set
#ifndef __AVR_ATmega2560__
  #error "Oops!  Make sure you have 'Arduino Mega 2560' selected from the 'Tools -> Boards' menu."
#endif

#define MAX_MOTORS                 (6)

#define MOTOR_0_DIR_PIN           (16)
#define MOTOR_0_STEP_PIN          (17)
#define MOTOR_0_ENABLE_PIN        (48)
#define MOTOR_0_LIMIT_SWITCH_PIN  (37)

#define MOTOR_1_DIR_PIN           (47)
#define MOTOR_1_STEP_PIN          (54)
#define MOTOR_1_ENABLE_PIN        (55)
#define MOTOR_1_LIMIT_SWITCH_PIN  (36)

#define MOTOR_2_DIR_PIN           (56)
#define MOTOR_2_STEP_PIN          (57)
#define MOTOR_2_ENABLE_PIN        (62)
#define MOTOR_2_LIMIT_SWITCH_PIN  (35)

#define MOTOR_3_DIR_PIN           (22)
#define MOTOR_3_STEP_PIN          (23)
#define MOTOR_3_ENABLE_PIN        (24)
#define MOTOR_3_LIMIT_SWITCH_PIN  (34)

#define MOTOR_4_DIR_PIN           (25)
#define MOTOR_4_STEP_PIN          (26)
#define MOTOR_4_ENABLE_PIN        (27)
#define MOTOR_4_LIMIT_SWITCH_PIN  (33)

#define MOTOR_5_DIR_PIN           (28)
#define MOTOR_5_STEP_PIN          (29)
#define MOTOR_5_ENABLE_PIN        (39)
#define MOTOR_5_LIMIT_SWITCH_PIN  (32)

#define MAX_BOARD_SERVOS          (1)
#define SERVO0_PIN                (5)

#define LIMIT_SWITCH_PIN_LEFT     (MOTOR_0_LIMIT_SWITCH_PIN)
#define LIMIT_SWITCH_PIN_RIGHT    (MOTOR_1_LIMIT_SWITCH_PIN)


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
#endif

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
#endif

#endif


#endif  // BOARD_RUMBA_H

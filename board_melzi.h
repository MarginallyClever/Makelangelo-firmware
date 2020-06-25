#ifndef BOARD_MELZI_H
  #define BOARD_MELZI_H
  //------------------------------------------------------------------------------
  // Makelangelo - firmware for various robot kinematic models
  // dan@marginallycelver.com 2013-12-26
  // Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
  //------------------------------------------------------------------------------


  //-------------------------------------------------------------------------------
  // Creality Melzi board (Ender3, 3Pro, CR-10, etc.)
  // Submitted by rob.m.kotz@gmail.com
  // Check out Marlin firmware for Melzi board pin definitions - 
  // https://github.com/MarlinFirmware/Marlin/blob/2.0.x/Marlin/src/pins/sanguino/pins_MELZI_CREALITY.h
  // -------------------------------------------------------------------------------


  #if MOTHERBOARD == BOARD_MELZI 

    #undef LCD_PINS_RS
    #undef LCD_PINS_ENABLE
    #undef LCD_PINS_D4
    #undef LCD_PINS_D5
    #undef LCD_PINS_D6
    #undef LCD_PINS_D7

    #define MAX_MOTORS                 (2)

    #define MOTOR_0_LETTER            'X'
    #define MOTOR_0_DIR_PIN           (21)
    #define MOTOR_0_STEP_PIN          (15)
    #define MOTOR_0_ENABLE_PIN        (14)
    #define MOTOR_0_LIMIT_SWITCH_PIN  (18)   /* X min */

    #define MOTOR_1_LETTER            'Y'
    #define MOTOR_1_DIR_PIN           (23)
    #define MOTOR_1_STEP_PIN          (22)
    #define MOTOR_1_ENABLE_PIN        (14)
    #define MOTOR_1_LIMIT_SWITCH_PIN  (19)  /* Y min */


    #define MAX_BOARD_SERVOS          (1)
    #define SERVO0_PIN                (27)   /* Servo 1 */

    #define LIMIT_SWITCH_PIN_LEFT     (MOTOR_0_LIMIT_SWITCH_PIN)
    #define LIMIT_SWITCH_PIN_RIGHT    (MOTOR_1_LIMIT_SWITCH_PIN)

    //#undef HAS_LCD
    #undef HAS_SD

      // LCD pins
    #ifdef LCD_IS_128X64
      // 128x64 full graphics controller
      #define BEEPER             -1

      #define LCD_PINS_RS        28
      #define LCD_PINS_ENABLE    17
      #define LCD_PINS_D4        30

      // Encoder rotation values
      #define BTN_EN1            11
      #define BTN_EN2            10
      #define BTN_ENC            16

      // SD card settings
      //#define SDSS               31
      //#define SDCARDDETECT       -1
      //#define SDPOWER            -1
      //#define KILL_PIN           -1


    #endif

  #endif

#endif  // BOARD_MELZI_H

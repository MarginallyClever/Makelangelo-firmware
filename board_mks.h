#ifndef BOARD_MKS_H
#define BOARD_MKS_H
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

// NOTE: MKS boards are identical to RAMPS boards, except that they have a few extra features.

#if MOTHERBOARD == BOARD_MKS

#define MAX_MOTORS                 (5)

#define MOTOR_0_DIR_PIN           (55)
#define MOTOR_0_STEP_PIN          (54)
#define MOTOR_0_ENABLE_PIN        (38)
#define MOTOR_0_LIMIT_SWITCH_PIN  (3)   /* X min */

#define MOTOR_1_DIR_PIN           (61)
#define MOTOR_1_STEP_PIN          (60)
#define MOTOR_1_ENABLE_PIN        (56)
#define MOTOR_1_LIMIT_SWITCH_PIN  (14)  /* Y min */

#define MOTOR_2_DIR_PIN           (48)
#define MOTOR_2_STEP_PIN          (46)
#define MOTOR_2_ENABLE_PIN        (62)
#define MOTOR_2_LIMIT_SWITCH_PIN  (18)  /* Z Min */

#define MOTOR_3_DIR_PIN           (28)
#define MOTOR_3_STEP_PIN          (26)
#define MOTOR_3_ENABLE_PIN        (24)
#define MOTOR_3_LIMIT_SWITCH_PIN  (2)   /* X Max */

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

// Smart controller settings
#define BEEPER             37   /* Pin on SMART Adapter */
#define LCD_PINS_RS        16   /* Pin on SMART Adapter */
#define LCD_PINS_ENABLE    17   /* Pin on SMART Adapter */ 
#define LCD_PINS_D4        23   /* Pin on SMART Adapter */
#define LCD_PINS_D5        25   /* Pin on SMART Adapter */
#define LCD_PINS_D6        27   /* Pin on SMART Adapter */
#define LCD_PINS_D7        29   /* Pin on SMART Adapter */

// Encoder rotation values
#define BTN_EN1            31   /* Pin on SMART Adapter */
#define BTN_EN2            33   /* Pin on SMART Adapter */
#define BTN_ENC            35  /* Pin on SMART Adapter */

#define KILL_PIN    41    /* Pin on SMART Adapter */

// SD card settings
#define SDPOWER            -1
#define SDSS               53
#define SDCARDDETECT       49

#define LCD_HEIGHT         4
#define LCD_WIDTH          20

#define BLEN_C             2
#define BLEN_B             1
#define BLEN_A             0
#define encrot0            0
#define encrot1            2
#define encrot2            3
#define encrot3            1

#endif  // BOARD_MKS


#endif  // BOARD_MKS_H


#ifndef CONFIGURE_H
#define CONFIGURE_H
//------------------------------------------------------------------------------
// Makelangelo - supports raprapdiscount RUMBA controller
// dan@marginallycelver.com 2013-12-26
// RUMBA should be treated like a MEGA 2560 Arduino.
//------------------------------------------------------------------------------
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/Makelangelo for more information.


//------------------------------------------------------------------------------
// Sanity check
//------------------------------------------------------------------------------

// wrong board type set
#ifndef __AVR_ATmega2560__
  #error "Oops!  Make sure you have 'Arduino Mega 2560' selected from the 'Tools -> Boards' menu."
#endif

//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------
//#define VERBOSE           (1)  // add to get a lot more serial output.


// Boards supported
#define BOARD_RUMBA        1
#define BOARD_RAMPS        2
#define BOARD_SANGUINOLULU 3
#define BOARD_TEENSYLU     4


// machine style - change this for your machine style.
//#define POLARGRAPH2  // uncomment this line if you use a polargraph like the Makelangelo 3 or 5
//#define COREXY  // uncomment this line if you use a CoreXY setup.
//#define TRADITIONALXY  // uncomment this line if you use a traditional XY setup.
#define ZARPLOTTER  // uncomment this line if you use a 4 motor Zarplotter


// servo angles for pen control
#define PEN_UP_ANGLE         (90)
#define PEN_DOWN_ANGLE       (50)  // Some steppers don't like 0 degrees

// for serial comms
#define BAUD                 (57600)  // How fast is the Arduino talking?
#define MAX_BUF              (64)  // What is the longest message Arduino can store?

#define MICROSTEPS           (16.0)  // microstepping on this microcontroller
#define STEPS_PER_TURN       (400.0 * MICROSTEPS)  // default number of steps per turn * microsteps

#define STEP_DELAY           (50)  // delay between steps, in milliseconds, when doing fixed tasks like homing

#ifdef POLARGRAPH2
//#define MAKELANGELO_HARDWARE_VERSION 3  // If you have a makelangelo 3+
#define MAKELANGELO_HARDWARE_VERSION 5  // If you have a makelangelo 5+

#define NUM_MOTORS           (3)  // Including servo
#define MAX_FEEDRATE         (9000.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (100)
#define MAX_JERK             (5.0)
#define DEFAULT_FEEDRATE     (7000.0)
#define DEFAULT_ACCELERATION (2500)


#if MAKELANGELO_HARDWARE_VERSION == 5
#define MOTHERBOARD BOARD_RUMBA 
#define USE_LIMIT_SWITCH    (1)  // Comment out this line to disable findHome and limit switches
#define HAS_SD                   // comment this out if there is no SD card
#define HAS_LCD                  // comment this out if there is no SMART LCD controller
#endif
#if MAKELANGELO_HARDWARE_VERSION == 3
#define MOTHERBOARD BOARD_RUMBA
#define HAS_SD                   // comment this out if there is no SD card
#define HAS_LCD                  // comment this out if there is no SMART LCD controller
#endif

#endif  // POLARGRAPH2


#ifdef ZARPLOTTER
#define MAKELANGELO_HARDWARE_VERSION 6
#define MOTHERBOARD BOARD_RUMBA

#define NUM_MOTORS           (5)  // Including servo
#define MAX_FEEDRATE         (15000.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (100)
#define MAX_JERK             (15.0)
#define DEFAULT_FEEDRATE     (10000.0)
#define DEFAULT_ACCELERATION (3500)

#define ZARPLOTTER_MOTOR_SIZE   (4.5f)
#define ZARPLOTTER_PLOTTER_SIZE (6.0f)
#define ZARPLOTTER_COMPENSATION (ZARPLOTTER_PLOTTER_SIZE/2.0f + ZARPLOTTER_MOTOR_SIZE)
#endif  // ZARPLOTTER


#define NUM_TOOLS            (6)
#define MAX_SEGMENTS         (32)  // number of line segments to buffer ahead. must be a power of two.
#define SEGMOD(x)            ((x)&(MAX_SEGMENTS-1))

// for arc directions
#define ARC_CW               (1)
#define ARC_CCW              (-1)
#define SEGMENT_PER_CM_LINE  (2)  // lines are split into segments.  How long are the segments?
#define SEGMENT_PER_CM_ARC   (3)  // Arcs are split into segments.  How long are the segments?


#ifdef HAS_LCD
#define HAS_SD
#endif

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


// Board pin layouts


#if MOTHERBOARD == BOARD_RUMBA 
#define MAX_MOTORS                 (6)

#define MOTOR_0_DIR_PIN           (16)
#define MOTOR_0_STEP_PIN          (17)
#define MOTOR_0_ENABLE_PIN        (48)
#define MOTOR_0_LIMIT_SWITCH_PIN  (37)

#define MOTOR_1_DIR_PIN           (47)
#define MOTOR_1_STEP_PIN          (54)
#define MOTOR_1_ENABLE_PIN        (55)
#define MOTOR_1_LIMIT_SWITCH_PIN  (36)

// alternate pins in case you want to do something interesting
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

#define NUM_SERVOS                (1)
#define SERVO0_PIN                (5)

#define LIMIT_SWITCH_PIN_LEFT     (MOTOR_0_LIMIT_SWITCH_PIN)
#define LIMIT_SWITCH_PIN_RIGHT    (MOTOR_1_LIMIT_SWITCH_PIN)

// Smart controller settings
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

#endif

#if MOTHERBOARD == BOARD_RAMPS 
#define MAX_MOTORS                 (5)

#define MOTOR_0_DIR_PIN           (55)
#define MOTOR_0_STEP_PIN          (54)
#define MOTOR_0_ENABLE_PIN        (38)
#define MOTOR_0_LIMIT_SWITCH_PIN  (3)   /* X min */

#define MOTOR_1_DIR_PIN           (61)
#define MOTOR_1_STEP_PIN          (60)
#define MOTOR_1_ENABLE_PIN        (56)
#define MOTOR_1_LIMIT_SWITCH_PIN  (14)  /* Y min */

// alternate pins in case you want to do something interesting
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

#define NUM_SERVOS         (4)
#define SERVO0_PIN         (11)   /* Servo 1 */
#define SERVO1_PIN         (6)
#define SERVO2_PIN         (5)
#define SERVO3_PIN         (4)

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
#endif

#if MOTHERBOARD == BOARD_SANGUINOLULU 
#define MAX_MOTORS                 (2)

#define MOTOR_0_DIR_PIN           (21)
#define MOTOR_0_STEP_PIN          (15)
#define MOTOR_0_ENABLE_PIN        (14)
#define MOTOR_0_LIMIT_SWITCH_PIN  (18)

#define MOTOR_1_DIR_PIN           (23)
#define MOTOR_1_STEP_PIN          (22)
#define MOTOR_1_ENABLE_PIN        (14)
#define MOTOR_1_LIMIT_SWITCH_PIN  (19)

// TODO: if ZARPLOTTER & SANGUINOLULU throw a compile error, not enough motors.

#define NUM_SERVOS         (1)
#define SERVO0_PIN         (12)
#endif

#if MOTHERBOARD == BOARD_TEENSYLU
#define MAX_MOTORS                 (2)

#define MOTOR_0_DIR_PIN           (29)
#define MOTOR_0_STEP_PIN          (28)
#define MOTOR_0_ENABLE_PIN        (19)
#define MOTOR_0_LIMIT_SWITCH_PIN  (26)

#define MOTOR_1_DIR_PIN           (31)
#define MOTOR_1_STEP_PIN          (30)
#define MOTOR_1_ENABLE_PIN        (18)
#define MOTOR_1_LIMIT_SWITCH_PIN  (27)

#define NUM_SERVOS                (1)
#define SERVO0_PIN                (24)

#endif


#if NUM_MOTORS > MAX_MOTORS
#error "The number of axies needed is more than this board supports."
#endif


//------------------------------------------------------------------------------
// EEPROM MEMORY MAP
//------------------------------------------------------------------------------
#define EEPROM_VERSION          7  // Increment EEPROM_VERSION when adding new variables
#define ADDR_VERSION            0                          // 0..255 (1 byte)
#define ADDR_UUID               (ADDR_VERSION+1)           // long - 4 bytes
#define ADDR_PULLEY_DIA1        (ADDR_UUID+4)              // float - 4 bytes
#define ADDR_PULLEY_DIA2        (ADDR_PULLEY_DIA1+4)       // float - 4 bytes unused?
#define ADDR_LEFT               (ADDR_PULLEY_DIA2+4)       // float - 4 bytes
#define ADDR_RIGHT              (ADDR_LEFT+4)              // float - 4 bytes
#define ADDR_TOP                (ADDR_RIGHT+4)             // float - 4 bytes
#define ADDR_BOTTOM             (ADDR_TOP+4)               // float - 4 bytes
#define ADDR_INVL               (ADDR_BOTTOM+4)            // bool - 1 byte
#define ADDR_INVR               (ADDR_INVL+1)              // bool - 1 byte
#define ADDR_HOMEX              (ADDR_INVR+1)              // float - 4 bytes
#define ADDR_HOMEY              (ADDR_HOMEX+4)             // float - 4 bytes
#define ADDR_CALIBRATION_LEFT   (ADDR_HOMEY+4)             // float - 4 bytes
#define ADDR_CALIBRATION_RIGHT  (ADDR_CALIBRATION_LEFT+4)  // float - 4 bytes
#define ADDR_INVU               (ADDR_CALIBRATION_RIGHT+4) // bool - 1 byte
#define ADDR_INVV               (ADDR_INVU+1)              // bool - 1 byte
#define ADDR_CALIBRATION_BLEFT  (ADDR_INVV+1)              // float - 4 bytes
#define ADDR_CALIBRATION_BRIGHT (ADDR_CALIBRATION_BLEFT+4) // float - 4 bytes


//------------------------------------------------------------------------------
// TIMERS
//------------------------------------------------------------------------------
// for timer interrupt control
#define CLOCK_FREQ            (16000000L)
#define MAX_COUNTER           (65536L)
// time passed with no instruction?  Make sure PC knows we are waiting.
#define TIMEOUT_OK            (1000)

// optimize code, please
#define FORCE_INLINE         __attribute__((always_inline)) inline


#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG;  cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START


//------------------------------------------------------------------------------
// STRUCTURES
//------------------------------------------------------------------------------
// for line()
typedef struct {
  long step_count;
  long delta;  // number of steps to move
  long absdelta;
  int dir;
  float delta_normalized;
} Axis;


typedef struct {
  int step_pin;
  int dir_pin;
  int enable_pin;
  int limit_switch_pin;
  int limit_switch_state;
  int reel_in;
  int reel_out;
} Motor;


typedef struct {
  Axis a[NUM_MOTORS];
  int steps_total;
  int steps_taken;
  int accel_until;
  int decel_after;
  unsigned short feed_rate_max;
  unsigned short feed_rate_start;
  unsigned short feed_rate_start_max;
  unsigned short feed_rate_end;
  char nominal_length_flag;
  char recalculate_flag;
  char busy;
} Segment;


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------

extern Segment line_segments[MAX_SEGMENTS];
extern Segment *working_seg;
extern volatile int current_segment;
extern volatile int last_segment;
extern float acceleration;
extern Motor motors[NUM_MOTORS];
extern const char *AxisLetters;

#endif // CONFIGURE_H


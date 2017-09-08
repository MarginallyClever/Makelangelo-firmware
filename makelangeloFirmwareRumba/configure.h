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


//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------
//#define VERBOSE           (1)  // add to get a lot more serial output.





// for serial comms
#define BAUD                 (57600)  // How fast is the Arduino talking?
#define MAX_BUF              (64)  // What is the longest message Arduino can store?

#define MICROSTEPS           (16.0)  // microstepping on this microcontroller
#define STEPS_PER_TURN       (400.0 * MICROSTEPS)  // default number of steps per turn * microsteps
#define STEP_DELAY           (50)  // delay between steps, in milliseconds, when doing fixed tasks like homing

#define MAX_ACCELERATION     (5000)
#define MIN_ACCELERATION     (100)

// machine style - change this for your machine style.
#define POLARGRAPH       1  // uncomment this line if you use a polargraph like the Makelangelo 3 or 5
//#define TRADITIONALXY 3 // uncomment this line if you use a traditional XY setup.
//#define COREXY        2 // uncomment this line if you use a CoreXY setup.
//#define ZARPLOTTER    4 // uncomment this line if you use a 4 motor Zarplotter

#include "robot_polargraph.h"
#include "robot_traditionalxy.h"
#include "robot_corexy.h"
#include "robot_zarplotter.h"

// buffering commands
#define MAX_SEGMENTS         (32)  // number of line segments to buffer ahead. must be a power of two.
#define SEGMOD(x)            ((x)&(MAX_SEGMENTS-1))

// for arc directions
#define ARC_CW               (1)
#define ARC_CCW              (-1)
#define SEGMENT_PER_CM_LINE  (2)  // lines are split into segments.  How long are the segments?
#define SEGMENT_PER_CM_ARC   (3)  // Arcs are split into segments.  How long are the segments?


// Boards supported
#define BOARD_RUMBA        1
#define BOARD_RAMPS        2
#define BOARD_SANGUINOLULU 3
#define BOARD_TEENSYLU     4

// Board descriptions
#include "board_rumba.h"
#include "board_ramps.h"
#include "board_sanguinolulu.h"
#include "board_teensylu.h"

// sanity checks
#if NUM_MOTORS > MAX_MOTORS
#error "The number of motors needed is more than this board supports."
#endif
#if NUM_SERVOS > MAX_BOARD_SERVOS
#error "The number of servos needed is more than this board supports."
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
} Motor;


typedef struct {
  Axis a[NUM_MOTORS+NUM_SERVOS];
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


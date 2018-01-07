#ifndef CONFIGURE_H
#define CONFIGURE_H
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------

//#define VERBOSE           (1)  // add to get a lot more serial output.




// robot styles supported
#define POLARGRAPH       1  // polargraph like Makelangelo
#define TRADITIONALXY    3  // gantry 2 axis setup.
#define COREXY           2  // gantry CoreXY setup.
#define ZARPLOTTER       4  // 4 motor, x-shaped 2D motion
#define SKYCAM           5  // 4 motor, x-shaped 3D motion
#define DELTA            6  // 3 arm delta robot, rotary action.  untested.
//#define KOSSEL           7  // 3 arm delta robot, linear action.  not supported yet.
#define STEWART          8  // 6 arm stewart platform, rotary action.  untested.
//#define OSGOODE          9  // 6 arm stewart platform, linear action.  not supported yet.
#define ARM3            10
//#define ARM4            11
//#define ARM5            12
#define ARM6            13

#define MACHINE_STYLE POLARGRAPH  // change this

// robot style descriptions
#include "robot_polargraph.h"
#include "robot_traditionalxy.h"
#include "robot_corexy.h"
#include "robot_zarplotter.h"
#include "robot_skycam.h"
#include "robot_delta.h"
//#include "robot_kossel.h"
#include "robot_stewart.h"
//#include "robot_osgoode.h"
#include "robot_arm3.h"
//#include "robot_arm4.h"
//#include "robot_arm5.h"
#include "robot_arm6.h"


// Boards supported
#define BOARD_RUMBA        1
#define BOARD_RAMPS        2
#define BOARD_SANGUINOLULU 3
#define BOARD_TEENSYLU     4

#define MOTHERBOARD BOARD_RUMBA  // change this

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


// for serial comms
#define BAUD                 (57600)  // How fast is the Arduino talking?
#define MAX_BUF              (64)  // What is the longest message Arduino can store?

// motor details
#define MICROSTEPS           (16.0)  // change this.  microstepping on this microcontroller
#define DEGREES_PER_STEP     ( 0.9)  // change this.  as advertised by the stepper motor maker

#define NORMAL_MOTOR_STEPS   (360.0/DEGREES_PER_STEP)  // 360/0.9=400.  360/1.8=200.
#define STEPS_PER_TURN       (NORMAL_MOTOR_STEPS * MICROSTEPS)  // default number of steps per turn * microsteps
#define PULLEY_PITCH         (2*20.0) // 2mm per tooth, 20 teeth.
#define THREAD_PER_STEP      (PULLEY_PITCH/STEPS_PER_TURN)
#define MICROSTEP_PER_DEGREE (STEPS_PER_TURN/360.0)

// buffering commands
#define MAX_SEGMENTS         (32)  // number of line segments to buffer ahead. must be a power of two.
#define SEGMOD(x)            ((x)&(MAX_SEGMENTS-1))

// for arc directions
#define ARC_CW               (1)
#define ARC_CCW              (-1)


//------------------------------------------------------------------------------
// LCD options
//------------------------------------------------------------------------------

#define LCD_MESSAGE_LENGTH (40)  // we have two lines of 20 characters avialable in 7.16
#define LCD_DRAW_DELAY     (150)
#define LCD_TURN_PER_MENU  (5)



//------------------------------------------------------------------------------
// EEPROM MEMORY MAP
//------------------------------------------------------------------------------
#define EEPROM_VERSION          8    // Increment when adding new variables
#define ADDR_VERSION            0                          // 0..255 (1 byte)
#define ADDR_UUID               (ADDR_VERSION+1)           // long - 4 bytes
#define ADDR_LIMITS             (ADDR_UUID+4)              // float - 4 bytes

#define ADDR_HOME               (ADDR_LIMITS+4*NUM_AXIES)  // float - 4 bytes

#define ADDR_CALIBRATION_LEFT   (ADDR_HOME+4*NUM_AXIES)    // float - 4 bytes
#define ADDR_CALIBRATION_RIGHT  (ADDR_CALIBRATION_LEFT+4)  // float - 4 bytes


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

typedef struct {
  float limitMax;
  float limitMin;
  float pos;
  float homePos;
} Axis;


// for line()
typedef struct {
  long step_count;
  long delta;  // number of steps to move
  long absdelta;
  int dir;
  float delta_normalized;
} SegmentAxis;


typedef struct {
  int step_pin;
  int dir_pin;
  int enable_pin;
  int limit_switch_pin;
} Motor;


typedef struct {
  SegmentAxis a[NUM_MOTORS+NUM_SERVOS];
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
extern const char *AxisNames;
extern const char *MotorNames;

extern char lcd_message[LCD_MESSAGE_LENGTH];


#endif // CONFIGURE_H


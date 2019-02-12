#ifndef CONFIGURE_H
#define CONFIGURE_H
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------

//#define VERBOSE           (1)  // add to get a lot more serial output.


//------------------------------------------------------------------------------
// Robot styles supported
//------------------------------------------------------------------------------

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

#define MACHINE_STYLE POLARGRAPH  // Change this

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

//------------------------------------------------------------------------------
// LCD panels supported
//------------------------------------------------------------------------------

#define HAS_LCD  // if you have an LCD panel
#define HAS_SD   // if you have SD card support on your LCD panel (must be on panel?)

// only uncomment one of these options
//#define LCD_IS_128X64  // reprapdiscount Full Graphic Smart LCD Controller
#define LCD_IS_SMART  // reprapdiscount Smart LCD Controller (including XXL model)

//------------------------------------------------------------------------------
// Microcontrollers supported
//------------------------------------------------------------------------------

#define BOARD_RUMBA        1
#define BOARD_RAMPS        2
#define BOARD_SANGUINOLULU 3
#define BOARD_TEENSYLU     4
#define BOARD_WEMOS        5

#define MOTHERBOARD BOARD_WEMOS  // change this

#include "board_rumba.h"
#include "board_ramps.h"
#include "board_sanguinolulu.h"
#include "board_teensylu.h"
#include "board_wemos.h"

//------------------------------------------------------------------------------
// MOTOR DETAILS
//------------------------------------------------------------------------------

// 400 step-per-turn motors move 0.9 degrees per step.  (360/400=0.9).  Marginallyclever.com default.
// 200 step-per-turn motors move 1.8 degrees per step.  (360/200=1.8)
// see your stepper motor data sheet for more info.
#define DEGREES_PER_STEP     ( 0.9)

// stepper motor drivers can use microstepping to split steps into fractions of steps for greater precision.
// A4988 drivers (Marginallyclever.com default) use 16x.
// DRV8825 can go up to 32x.
// TMC2130 can go to 256x.
// see your driver data sheet.
// note that some boards have dip switches or jumpers that can be activated to turn stepping on and off.
// make sure your dip switch settings match the firmware value.
#define MICROSTEPS           (16.0)

// Marginallyclever.com uses GT2 timing belt, which has 2mm teeth.
// We also use GT2-20 pulleys which have 20 teeth.
// 20*2 means the pitch is 40.
#define PULLEY_PITCH         (40.0)

// These numbers are calculated from the above.  No need to change these.
#define NORMAL_MOTOR_STEPS   (360.0/DEGREES_PER_STEP)
#define STEPS_PER_TURN       (NORMAL_MOTOR_STEPS * MICROSTEPS)
#define THREAD_PER_STEP      (PULLEY_PITCH/STEPS_PER_TURN)
#define MICROSTEP_PER_DEGREE (STEPS_PER_TURN/360.0)

//------------------------------------------------------------------------------
// COMMUNICATION & BUFFERING
//------------------------------------------------------------------------------
// for serial
#define BAUD                 (57600)  // How fast is the Arduino talking?
#define MAX_BUF              (64)  // What is the longest message Arduino can store?

// buffering commands
#ifndef MAX_SEGMENTS
#define MAX_SEGMENTS         (32)  // number of line segments to buffer ahead. must be a power of two.
#endif

#define SEGMOD(x)            ((x)&(MAX_SEGMENTS-1))

//------------------------------------------------------------------------------
// MISC
//------------------------------------------------------------------------------

// for arc directions
#define ARC_CW               (1)
#define ARC_CCW              (-1)

//------------------------------------------------------------------------------
// SANITY CHECKS
//------------------------------------------------------------------------------

#if NUM_MOTORS > MAX_MOTORS
#error "The number of motors needed is more than this board supports."
#endif
#if NUM_SERVOS > MAX_BOARD_SERVOS
#error "The number of servos needed is more than this board supports."
#endif

#if NUM_SERVOS + NUM_MOTORS != NUM_AXIES
// not always the case!  Skycam has more motors than axies.  
//#error "NUM_SERVOS + NUM_MOTORS != NUM_AXIES"
#endif

//------------------------------------------------------------------------------
// EEPROM MEMORY MAP
//------------------------------------------------------------------------------
#define FIRMWARE_VERSION        9    // Increment when adding new variables
#define ADDR_VERSION            0                          // 0..255 (1 byte)
#define ADDR_UUID               (ADDR_VERSION+1)           // long - 4 bytes
#define ADDR_LIMITS             (ADDR_UUID+4)              // float - 4 bytes

#define ADDR_HOME               (ADDR_LIMITS+4*NUM_AXIES)  // float - 4 bytes

#define ADDR_CALIBRATION_LEFT   (ADDR_HOME+4*NUM_AXIES)    // float - 4 bytes
#define ADDR_CALIBRATION_RIGHT  (ADDR_CALIBRATION_LEFT+4)  // float - 4 bytes


//------------------------------------------------------------------------------
// TIMERS
//------------------------------------------------------------------------------

#ifdef ESP8266

#define CLOCK_FREQ            (80000000L)
#define MAX_COUNTER           (4294967295L)  // 32 bits

// optimize code, please
#define FORCE_INLINE         __attribute__((always_inline)) inline


#define CRITICAL_SECTION_START  noInterrupts();
#define CRITICAL_SECTION_END    interrupts();

#else  // ESP8266

// for timer interrupt control
#define CLOCK_FREQ            (16000000L)
#define MAX_COUNTER           (65536L)  // 16 bits

// optimize code, please
#define FORCE_INLINE         __attribute__((always_inline)) inline

#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG;  cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START

#endif  // ESP8266


// TODO a guess.  use real math here!
// https://reprap.org/wiki/Step_rates
// 0.9deg stepper, 20-tooth GT2 pulley, 1/16 microstepping = 160 steps/mm, 1500mm/s = 240000 steps/s
#define CLOCK_MAX_STEP_FREQUENCY (240000)
#define CLOCK_MIN_STEP_FREQUENCY (CLOCK_FREQ/500000U)


//------------------------------------------------------------------------------
// STRUCTURES
//------------------------------------------------------------------------------

typedef struct {
  float limitMax;
  float limitMin;
  float pos;
  float homePos;
} Axis;


#include "motor.h"

//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------

extern float feed_rate;
extern float acceleration;
extern float step_delay;
extern int robot_uid;
extern Axis axies[NUM_AXIES];
extern float calibrateRight;
extern float calibrateLeft;
extern char serialBuffer[MAX_BUF + 1]; // Serial buffer
extern int sofar;                      // Serial buffer progress

extern void pause(const long us);
extern void findStepDelay();
extern void IK(const float *const axies, long *motorStepArray);
extern int FK(long *motorStepArray,float *axies);
extern void robot_findHome();
extern void robot_setup();
extern void processCommand();
extern void parser_ready();
extern void teleport(float *pos);
extern void lineSafe(float *pos, float new_feed_rate);
extern void get_end_plus_offset(float *results);
extern void set_tool_offset(int toolID, float *pos);
extern void reportCalibration();
extern void where();

#endif // CONFIGURE_H

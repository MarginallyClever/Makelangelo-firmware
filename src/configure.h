#pragma once
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------

//#define VERBOSE           (1)  // add to get a lot more serial output.
//#define DEBUG_STEPPING         // uncomment to debug stepper internal timer
#ifdef DEBUG_STEPPING
//#  define DEBUG_STEP_TIMING  // how many ticks per 
#endif

#include "boards/boards.h"
#include "robots/robots.h"

//------------------------------------------------------------------------------
// LCD panels supported
//------------------------------------------------------------------------------

#define LCD_NONE      0
#define LCD_IS_128X64 1  // reprapdiscount Full Graphic Smart LCD Controller
#define LCD_IS_SMART  2  // reprapdiscount Smart LCD Controller (including XXL model)

//#define LCD_TYPE LCD_NONE

#if __has_include("local_config.h")
// Your local changes go here.
// Do not send your local_config.h in a pull request.  Thank you!
#  include "local_config.h"
#endif

//------------------------------------------------------------------------------
// SANITY CHECKS 1
//------------------------------------------------------------------------------

#ifndef MOTHERBOARD
#error No MOTHERBOARD defined.
#endif

#ifndef MACHINE_STYLE
#error No MACHINE_STYLE defined.
#endif

#if MACHINE_STYLE==STEWART_ROTARY || MACHINE_STYLE==STEWART_LINEAR
#define IS_STEWART_PLATFORM
#endif

#if defined(LCD_TYPE)
  #if LCD_TYPE!=LCD_NONE
  #define HAS_LCD
  #else
    #ifdef HAS_LCD
    #error LCD_TYPE==LCD_NONE *and* HAS_LCD?
    #endif
  #endif
#endif

//------------------------------------------------------------------------------

#define NUM_MUSCLES (NUM_MOTORS + NUM_SERVOS)

// after this many milliseconds with no serial communication, the robot pings anyone connected by calling ready().
#define TIMEOUT_OK (2000)


#include "config_motors.h"
#include <Arduino.h>

extern void meanwhile();
#include "planner.h"

#include "motor.h"
#include "parser.h"
#include "eeprom_manager.h"
#include "gripper_hande.h"
#include "tests.h"

//------------------------------------------------------------------------------
// SANITY CHECKS 2
//------------------------------------------------------------------------------

#if NUM_MOTORS > MAX_MOTORS
#  error "The number of motors needed is more than this board supports."
#endif
#if NUM_SERVOS > MAX_BOARD_SERVOS
#  error "The number of servos needed is more than this board supports."
#endif
#if NUM_MUSCLES != NUM_AXIES
// not always the case!  Skycam has more motors than axies.
//#error "NUM_MUSCLES != NUM_AXIES"
#endif
#if defined(HAS_LCD) && (!defined(LCD_TYPE) || LCD_TYPE==NONE)
#  error "If HAS_LCD is defined then but the type is undefined."
#endif

//------------------------------------------------------------------------------
// STRUCTURES
//------------------------------------------------------------------------------

typedef struct {
  float limitMax;
  float limitMin;
  float pos;
  float homePos;
} Axis;

//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------

// eeprom values
extern int robot_uid;
extern float calibrateRight;
extern float calibrateLeft;

extern float desiredFeedRate;
extern float desiredAcceleration;
extern Axis axies[NUM_AXIES];

extern void pause(const uint32_t us);
extern hal_timer_t findStepDelay();

extern void IK(const float *const axies, int32_t *motorStepArray);
extern int FK(int32_t *motorStepArray, float *axies);

extern void robot_findHome();
extern void robot_setup();
extern void teleport(float *pos);
extern void get_end_plus_offset(float *results);
extern void set_tool_offset(int toolID, float *pos);
extern void reportCalibration();
extern void setHome(float *pos);

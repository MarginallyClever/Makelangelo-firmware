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

//------------------------------------------------------------------------------
// Robot styles supported
//------------------------------------------------------------------------------

#define POLARGRAPH       1  // polargraph like Makelangelo
#define TRADITIONALXY    3  // gantry 3 axis setup.
#define COREXY           2  // gantry CoreXY setup.
#define ZARPLOTTER       4  // 4 motor, x-shaped 2D motion
#define SKYCAM           5  // 4 motor, x-shaped 3D motion
#define DELTA            6  // 3 arm delta robot, rotary action.  untested.
#define STEWART_ROTARY   7  // Stewart platform: 6 arm stewart platform, rotary action.  untested.
#define ARM3             8  // Arm3: 3DOF palletizing robot arm.
#define SIXI             9  // Sixi: 6DOF robot arm.
#define TRADITIONAL6    10  // Traditional6: 6 axis machine, no restrictions.
#define SCARA           11  // SCARA: two link, two joint, 2D motion
#define SIXI3           12  // Sixi 3 robot arm.  5-6 DOF + Servo
#define STEWART_LINEAR  13  // Stewart platform: 6 arm stewart platform, linear action.

// default value
// !! this is now handeled via pio.ini !!
#ifndef MACHINE_STYLE
#error No machine style defined.
#endif

#if MACHINE_STYLE==STEWART_ROTARY || MACHINE_STYLE==STEWART_LINEAR
#define IS_STEWART_PLATFORM
#endif

//------------------------------------------------------------------------------
// LCD panels supported
//------------------------------------------------------------------------------

#define LCD_NONE      0
#define LCD_IS_128X64 1  // reprapdiscount Full Graphic Smart LCD Controller
#define LCD_IS_SMART  2  // reprapdiscount Smart LCD Controller (including XXL model)

//#define LCD_TYPE LCD_NONE

#if defined(LCD_TYPE) && LCD_TYPE!=LCD_NONE
#define HAS_LCD
#endif

//------------------------------------------------------------------------------
// Microcontrollers supported
//------------------------------------------------------------------------------

#define BOARD_RUMBA        1  // Reprap discount Rumba board
#define BOARD_RAMPS        2  // Mega2560 + Ramps 1.4
#define BOARD_SANGUINOLULU 3  // Sanguinolulu
#define BOARD_TEENSYLU     4  // Teensylu
#define BOARD_WEMOS        5  // Wemos D1 R2 + CNC Shield v3 (see board_wemos.h)
#define BOARD_SIXI_MEGA    6  // Arduino Mega + custom shield for Sixi 2 robot
#define BOARD_CNCV3        7  // Mega2560 + CNC Shield v3
#define BOARD_ESP32        8  // ESP32 + Marginally Clever Polargraph PCB.
#define BOARD_SKRPRO1_2    9  // SKR Pro 1.2

// default value 
// !! this is now handeled via platformio.ini !!
#ifndef MOTHERBOARD
#define MOTHERBOARD BOARD_RUMBA
#endif

//------------------------------------------------------------------------------
// YOUR CHANGES GO HERE
//------------------------------------------------------------------------------

#if __has_include("local_config.h")
// Your local changes go here.
// Do not send your local_config.h in a pull request.  Thank you!
#  include "local_config.h"
#endif

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------

#include "boards/rumba.h"
#include "boards/ramps.h"
#include "boards/sanguinolulu.h"
#include "boards/teensylu.h"
#include "boards/wemos.h"
#include "boards/sixi_mega.h"
#include "boards/cncv3.h"
#include "boards/esp32.h"
#include "boards/skrpro.h"

#include "robots/polargraph.h"
#include "robots/traditionalxy.h"
#include "robots/corexy.h"
#include "robots/zarplotter.h"
#include "robots/skycam.h"
#include "robots/delta.h"
#include "robots/stewartRotary.h"
#include "robots/arm3.h"
#include "robots/sixi.h"
#include "robots/traditional6.h"
#include "robots/scara.h"
#include "robots/sixi3.h"
#include "robots/stewartLinear.h"

#define NUM_MUSCLES (NUM_MOTORS + NUM_SERVOS)

#include "config_motors.h"
#include <Arduino.h>
#include "clock.h"

extern void meanwhile();
#include "planner.h"

#include "motor.h"
#include "parser.h"
#include "eeprom_manager.h"
#include "gripper_hande.h"
#include "tests.h"

//------------------------------------------------------------------------------
// SANITY CHECKS
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

extern void IK(const float *const axies, long *motorStepArray);
extern int FK(long *motorStepArray, float *axies);

extern void robot_findHome();
extern void robot_setup();
extern void teleport(float *pos);
extern void get_end_plus_offset(float *results);
extern void set_tool_offset(int toolID, float *pos);
extern void reportCalibration();
extern void setHome(float *pos);

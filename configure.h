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
#define STEWART          7  // 6 arm stewart platform, rotary action.  untested.
#define ARM3             8  // 3DOF palletizing robot arm.
#define SIXI             9  // 6DOF robot arm.
#define TRADITIONAL6    10  // 6 axis machine, no restrictions.
#define SCARA           11  // 2 axis SCARA.

// default value
#define MACHINE_STYLE POLARGRAPH

//------------------------------------------------------------------------------
// LCD panels supported
//------------------------------------------------------------------------------

#define LCD_NONE       0
#define LCD_IS_128X64  1  // reprapdiscount Full Graphic Smart LCD Controller
#define LCD_IS_SMART   2  // reprapdiscount Smart LCD Controller (including XXL model)

// default value
#define LCD_TYPE LCD_IS_SMART

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

// default value
#define MOTHERBOARD BOARD_RUMBA

//------------------------------------------------------------------------------
// YOUR CHANGES GO HERE
//------------------------------------------------------------------------------

#if __has_include("local_config.h")
// Your local changes go here.
// Do not send your local_config.h in a pull request.  Thank you!
#include "local_config.h"
#endif


//------------------------------------------------------------------------------
// 
//------------------------------------------------------------------------------

#include "robot_polargraph.h"
#include "robot_traditionalxy.h"
#include "robot_corexy.h"
#include "robot_zarplotter.h"
#include "robot_skycam.h"
#include "robot_delta.h"
#include "robot_stewart.h"
#include "robot_arm3.h"
#include "robot_sixi.h"
#include "robot_traditional6.h"
#include "robot_scara.h"

#include "board_rumba.h"
#include "board_ramps.h"
#include "board_sanguinolulu.h"
#include "board_teensylu.h"
#include "board_wemos.h"
#include "board_sixi_mega.h"
#include "board_cncv3.h"
#include "board_esp32.h"

#include "configMotors.h"

#include <Arduino.h>

#include "clock.h"
#include "motor.h"
#include "parser.h"
#include "eepromManager.h"

#include "gripper_hande.h"

//------------------------------------------------------------------------------
// SANITY CHECKS
//------------------------------------------------------------------------------

#if NUM_MOTORS > MAX_MOTORS
#error "The number of motors needed is more than this board supports."
#endif
#if NUM_SERVOS > MAX_BOARD_SERVOS
#error "The number of servos needed is more than this board supports."
#endif
#if NUM_MUSCLES != NUM_AXIES
// not always the case!  Skycam has more motors than axies.
//#error "NUM_MUSCLES != NUM_AXIES"
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

extern float feed_rate;
extern float acceleration;
extern float step_delay;
extern Axis axies[NUM_AXIES];


extern void pause(const long us);
extern void findStepDelay();

extern void IK(const float *const axies, long *motorStepArray);
extern int FK(long *motorStepArray, float *axies);

extern void robot_findHome();
extern void robot_setup();
extern void teleport(float *pos);
extern void lineSafe(float *pos, float new_feed_rate);
extern void arc(float cx, float cy, float *destination, char clockwise, float new_feed_rate);
extern void get_end_plus_offset(float *results);
extern void set_tool_offset(int toolID, float *pos);
extern void reportCalibration();
extern void meanwhile();
extern void setHome(float *pos);

#pragma once

// Robot styles supported.  One define per machine style.
// Define one of these in platformio.ini.  example: -DMACHINE_STYLE=POLARGRAPH

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


#include "polargraph.h"
#include "traditionalxy.h"
#include "corexy.h"
#include "zarplotter.h"
#include "skycam.h"
#include "delta.h"
#include "stewartRotary.h"
#include "arm3.h"
#include "sixi.h"
#include "traditional6.h"
#include "scara.h"
#include "sixi3.h"
#include "stewartLinear.h"

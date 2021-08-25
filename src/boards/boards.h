#pragma once

// Microcontroller Hardware Access Layer (HAL) supported.
// Define one of these in platformio.ini.  example: -DMOTHERBOARD=BOARD_RUMBA

#define BOARD_RUMBA        1  // Reprap discount Rumba board
#define BOARD_RAMPS        2  // Mega2560 + Ramps 1.4
#define BOARD_SANGUINOLULU 3  // Sanguinolulu
#define BOARD_TEENSYLU     4  // Teensylu
#define BOARD_WEMOS        5  // Wemos D1 R2 + CNC Shield v3 (see board_wemos.h)
#define BOARD_SIXI_MEGA    6  // Arduino Mega + custom shield for Sixi 2 robot
#define BOARD_CNCV3        7  // Mega2560 + CNC Shield v3
#define BOARD_ESP32        8  // ESP32 + Marginally Clever Polargraph PCB.
#define BOARD_SKRPRO1_2    9  // SKR Pro 1.2


#include "rumba.h"
#include "ramps.h"
#include "sanguinolulu.h"
#include "teensylu.h"
#include "wemos.h"
#include "sixi_mega.h"
#include "cncv3.h"
#include "esp32.h"
#include "skrpro.h"

#ifndef LIMIT_SWITCH_PRESSED_LEFT 
#define LIMIT_SWITCH_PRESSED_LEFT LOW
#endif

#ifndef LIMIT_SWITCH_PRESSED_RIGHT 
#define LIMIT_SWITCH_PRESSED_RIGHT LOW
#endif

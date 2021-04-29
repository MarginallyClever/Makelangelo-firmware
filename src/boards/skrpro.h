#pragma once
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

// SKR PRO 1.2
// In platformIO, when building any goal starting with "env:BIGTREE_SKR_PRO-*" the output will be found at
// .Makelangelo-firmware\.pio\build\BIGTREE_SKR_PRO-*\firmware.bin.
// This file is then copied to the SD card, moved to the board, and then the board is rebooted.
// firmware.bin will be changed to FIRMWARE.CUR.

#if MOTHERBOARD == BOARD_SKRPRO1_2

// wrong board type set
#  ifndef HEADER_SKRPRO
#    error "Oops!  Make sure you build this project using PlatformIO"
#  endif

#  define CPU_32_BIT
#  define CLOCK_FREQ (168000000L)

#  define MAX_MOTORS (6)

//
// Trinamic Stallguard pins
//
#define X_DIAG_PIN                          PB10  // X-
#define Y_DIAG_PIN                          PE12  // Y-
#define Z_DIAG_PIN                          PG8   // Z-
#define E0_DIAG_PIN                         PE15  // E0
#define E1_DIAG_PIN                         PE10  // E1
#define E2_DIAG_PIN                         PG5   // E2


// pin outs, see Makelangelo-firmware\platformio\variants\BIGTREE_SKR_PRO_1v1\pinout.png
#  define MOTOR_0_LETTER           'X'
#  define MOTOR_0_STEP_PIN         (PE9)
#  define MOTOR_0_DIR_PIN          (PF1)
#  define MOTOR_0_ENABLE_PIN       (PF2)
#  define MOTOR_0_LIMIT_SWITCH_PIN (PA15)

#  define MOTOR_1_LETTER           'Y'
#  define MOTOR_1_STEP_PIN         (PE11)
#  define MOTOR_1_DIR_PIN          (PE8)
#  define MOTOR_1_ENABLE_PIN       (PD7)
#  define MOTOR_1_LIMIT_SWITCH_PIN (PB8)

#  define MOTOR_2_LETTER           'Z'
#  define MOTOR_2_STEP_PIN         (PE13)
#  define MOTOR_2_DIR_PIN          (PC2)
#  define MOTOR_2_ENABLE_PIN       (PC0)
#  define MOTOR_2_LIMIT_SWITCH_PIN (PB9)

#  define MOTOR_3_LETTER           'U'
#  define MOTOR_3_STEP_PIN         (PE14)
#  define MOTOR_3_DIR_PIN          (PA0)
#  define MOTOR_3_ENABLE_PIN       (PC3)
#  define MOTOR_3_LIMIT_SWITCH_PIN (PB3)

#  define MOTOR_4_LETTER           'V'
#  define MOTOR_4_STEP_PIN         (PD15)
#  define MOTOR_4_DIR_PIN          (PE7)
#  define MOTOR_4_ENABLE_PIN       (PA3)
#  define MOTOR_4_LIMIT_SWITCH_PIN (PG15)

#  define MOTOR_5_LETTER           'W'
#  define MOTOR_5_STEP_PIN         (PD13)
#  define MOTOR_5_DIR_PIN          (PG9)
#  define MOTOR_5_ENABLE_PIN       (PF0)
#  define MOTOR_5_LIMIT_SWITCH_PIN (PG12)

#  define MAX_BOARD_SERVOS (1)
#  define SERVO0_PIN       (5)

#  define LIMIT_SWITCH_PIN_LEFT  (MOTOR_0_LIMIT_SWITCH_PIN)
#  define LIMIT_SWITCH_PIN_RIGHT (MOTOR_1_LIMIT_SWITCH_PIN)

// LCD pins
#  if LCD_TYPE == LCD_IS_128X64
#    define BEEPER          44
#    define LCD_PINS_RS     19
#    define LCD_PINS_ENABLE 42
#    define LCD_PINS_D4     18
#    define LCD_PINS_D5     38
#    define LCD_PINS_D6     41
#    define LCD_PINS_D7     40

// Encoder rotation values
#    define BTN_EN1 11
#    define BTN_EN2 12
#    define BTN_ENC 43

// SD card settings
#    define SDPOWER      -1
#    define SDSS         53
#    define SDCARDDETECT 49
#  endif

#  if LCD_TYPE == LCD_IS_SMART
#    define BEEPER          44
#    define LCD_PINS_RS     19
#    define LCD_PINS_ENABLE 42
#    define LCD_PINS_D4     18
#    define LCD_PINS_D5     38
#    define LCD_PINS_D6     41
#    define LCD_PINS_D7     40

// Encoder rotation values
#    define BTN_EN1 11
#    define BTN_EN2 12
#    define BTN_ENC 43

// SD card settings
#    define SDPOWER      -1
#    define SDSS         53
#    define SDCARDDETECT 49
#  endif

#  ifdef HAS_TMC2130

#    define CS_PIN_0 30  // chip select
#    define CS_PIN_1 31  // chip select

#  endif  // HAS_TMC2130

// STM32F4xx specific
#if defined(STM32F4xx)
#define STEP_TIMER 6
#define HAL_TIMER_RATE (F_CPU/2)
#endif

#define STEPPER_TIMER_RATE 2000000 // 2 Mhz
#define STEPPER_TIMER_PRESCALE ((HAL_TIMER_RATE)/(STEPPER_TIMER_RATE))
#define STEPPER_TIMER_TICKS_PER_US ((STEPPER_TIMER_RATE) / 1000000) // stepper timer ticks per µs

#include "HAL_stm32.h"

#endif

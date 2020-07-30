#pragma once


//------------------------------------------------------------------------------
// TIMERS
//------------------------------------------------------------------------------

#ifdef ESP8266

#define MAX_COUNTER             (4294967295L)  // 32 bits
#define CRITICAL_SECTION_START  noInterrupts();
#define CRITICAL_SECTION_END    interrupts();

#else  // ESP8266

extern void CRITICAL_SECTION_START();
extern void CRITICAL_SECTION_END();

// for timer interrupt control
#define MAX_COUNTER             (65536L)  // 16 bits

#endif  // ESP8266

#ifndef CLOCK_FREQ
#define CLOCK_FREQ                (16000000L)
#endif

#define TIMER_RATE            ((CLOCK_FREQ)/8)

// TODO a guess.  use real math here!
// https://reprap.org/wiki/Step_rates
// 0.9deg stepper @ 1/16 microstepping = 6400 steps/turn.  w/ 20-tooth GT2 pulley, 6400 steps = 40mm. 
// 1M us / 6400 = 156us/step.  100mm/s would be 2.5x faster, or 62.4us/step.  not much!
//#define CLOCK_MAX_STEP_FREQUENCY (240000L)
#define CLOCK_MIN_STEP_FREQUENCY (CLOCK_FREQ/500000U)

#define TIMEOUT_OK (1000)

// uncomment this to slow the machine and smooth movement if the segment buffer is running low.
#define BUFFER_EMPTY_SLOWDOWN
#ifndef MIN_SEGMENT_TIME_US
#define MIN_SEGMENT_TIME_US  (25000)
#endif

// if a segment added to the buffer is less tahn this many motor steps, roll it into the next move.
#define MIN_STEPS_PER_SEGMENT 6

#define MINIMUM_PLANNER_SPEED 0.05 // (mm/s)

#ifndef MAX_OCR1A_VALUE
#define MAX_OCR1A_VALUE (0xFFFF)
#endif

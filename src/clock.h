#pragma once

//------------------------------------------------------------------------------
// TIMERS
//------------------------------------------------------------------------------

// TODO a guess.  use real math here!
// https://reprap.org/wiki/Step_rates
// 0.9deg stepper @ 1/16 microstepping = 6400 steps/turn.  w/ 20-tooth GT2 pulley, 6400 steps = 40mm.
// 1M us / 6400 = 156us/step.  100mm/s would be 2.5x faster, or 62.4us/step.  not much!
//#define CLOCK_MAX_STEP_FREQUENCY (240000L)
#define CLOCK_MIN_STEP_FREQUENCY (STEPPER_TIMER_RATE / 500000U)

#define TIMEOUT_OK (1000)

// uncomment this to slow the machine and smooth movement if the segment buffer is running low.
// if the segment buffer runs empty the machine will be forced to stop.  if the machine slows just enough
// for the buffer to keep filling up then the machine will never come to a complete stop.  end result smoother movement and maybe shorter drawing time.
#define BUFFER_EMPTY_SLOWDOWN
#ifndef MIN_SEGMENT_TIME_US
#  define MIN_SEGMENT_TIME_US (25000)
#endif

// if a segment added to the buffer is less than this many motor steps, roll it into the next move.
#define MIN_STEPS_PER_SEGMENT 6

#define MINIMUM_PLANNER_SPEED 0.05  // (mm/s)

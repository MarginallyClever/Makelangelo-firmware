#pragma once

//------------------------------------------------------------------------------
// TIMERS
//------------------------------------------------------------------------------

// TODO a guess.  use real math here!
// https://reprap.org/wiki/Step_rates
#define CLOCK_MIN_STEP_FREQUENCY (F_CPU / 500000U)

#define TIMEOUT_OK (1000)

// uncomment this to slow the machine and smooth movement if the segment buffer is running low.
// if the segment buffer runs empty the machine will be forced to stop.  if the machine slows just enough
// for the buffer to keep filling up then the machine will never come to a complete stop.  end result smoother movement and maybe shorter drawing time.
#define BUFFER_EMPTY_SLOWDOWN
#ifndef DEFAULT_MIN_SEGMENT_TIME_US
#  define DEFAULT_MIN_SEGMENT_TIME_US (25000)
#endif

// if a segment added to the buffer is less than this many motor steps, roll it into the next move.
#define MIN_STEPS_PER_SEGMENT 6

#define MINIMUM_PLANNER_SPEED 0.05  // (mm/s)

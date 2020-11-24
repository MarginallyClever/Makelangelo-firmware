#pragma once
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#include <Arduino.h>

#include "macros.h"
#include "tmc2130.h"


//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------

#ifdef HAS_TMC2130
#else
// A4988
#define STEPPER_DIR_HIGH   HIGH
#define STEPPER_DIR_LOW    LOW
#endif


#define NUM_MUSCLES (NUM_MOTORS+NUM_SERVOS)

//------------------------------------------------------------------------------
// MOVE BUFFERING
//------------------------------------------------------------------------------


// buffering commands
#ifndef MAX_SEGMENTS
#define MAX_SEGMENTS         (32)  // number of line segments to buffer ahead. must be a power of two.
#endif

#define SEGMOD(x)            ((x)&(MAX_SEGMENTS-1))


//------------------------------------------------------------------------------
// step direction
//------------------------------------------------------------------------------

#ifndef START0
#define START0 LOW
#endif
#ifndef START1
#define START1 LOW
#endif
#ifndef START2
#define START2 LOW
#endif
#ifndef START3
#define START3 LOW
#endif
#ifndef START4
#define START4 LOW
#endif
#ifndef START5
#define START5 LOW
#endif

#ifndef END0
#define END0 HIGH
#endif
#ifndef END1
#define END1 HIGH
#endif
#ifndef END2
#define END2 HIGH
#endif
#ifndef END3
#define END3 HIGH
#endif
#ifndef END4
#define END4 HIGH
#endif
#ifndef END5
#define END5 HIGH
#endif

//------------------------------------------------------------------------------
// STRUCTURES
//------------------------------------------------------------------------------

typedef struct {
  char letter;
  uint8_t step_pin;
  uint8_t dir_pin;
  uint8_t enable_pin;
  uint8_t limit_switch_pin;
} Motor;


// for line()
typedef struct {
  int32_t step_count;  // current motor position, in steps.
  int32_t delta_steps;  // steps
  int32_t delta_mm;  // mm
  uint32_t absdelta;
  int dir;
#if MACHINE_STYLE == SIXI
  float expectedPosition;
  float positionStart;
  float positionEnd;
#endif
} Muscle;


typedef struct {
  Muscle a[NUM_MUSCLES];

  float distance;         // mm
  float nominal_speed;    // mm/s
  float entry_speed;      // mm/s
  float entry_speed_max;  // mm/s
  float acceleration;     // mm/sec^2
  
  uint32_t steps_total;    // steps
  uint32_t steps_taken;    // steps
  uint32_t accel_until;    // steps
  uint32_t decel_after;    // steps
  
  uint32_t nominal_rate;   // steps/s
  uint32_t initial_rate;     // steps/s
  uint32_t final_rate;      // steps/s
  uint32_t acceleration_steps_per_s2;  // steps/s^2
  uint32_t acceleration_rate;  // ?
  
  char nominal_length_flag;
  char recalculate_flag;
  char busy;
} Segment;


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------

extern Segment line_segments[MAX_SEGMENTS];
extern Segment *working_seg;
extern volatile int current_segment, last_segment, nonbusy_segment;
extern int first_segment_delay;
extern Motor motors[NUM_MOTORS];
extern const char *AxisNames;
extern const char *MotorNames;
extern float max_jerk[NUM_MUSCLES];
extern float max_feedrate_mm_s[NUM_MUSCLES];

extern uint32_t min_segment_time_us;

//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------

extern void motor_set_step_count(long *a);
extern void wait_for_empty_segment_buffer();
extern char segment_buffer_full();
extern void motor_line(const float * const target_position,float fr_mm_s,float millimeters);
extern void motor_engage();
extern void motor_home();
extern void enable_stealthChop();
extern void motor_disengage();
extern void motor_setup();
extern void setPenAngle(int arg0);
extern void clockISRProfile();

extern const int movesPlanned();
extern void motor_onestep(int motor);

#ifdef DEBUG_STEPPING
extern void debug_stepping();
#endif // DEBUG_STEPPING

FORCE_INLINE const int movesPlanned() {
  return SEGMOD( last_segment - current_segment );
}

FORCE_INLINE const int movesPlannedNotBusy() {
  return SEGMOD( last_segment - nonbusy_segment );
}

FORCE_INLINE const int get_next_segment(int i) {
  return SEGMOD( i + 1 ); 
}

FORCE_INLINE const int get_prev_segment(int i) {
  return SEGMOD( i - 1 );
}

FORCE_INLINE Segment *get_current_segment() {
  if (current_segment == last_segment ) return NULL;
  if (first_segment_delay > 0) {
    --first_segment_delay;
    if (movesPlanned() > 3) first_segment_delay = 0;
    return NULL;
  }

  Segment *block = &line_segments[current_segment];
  
  if(block->recalculate_flag!=0) return NULL;  // wait, not ready
  nonbusy_segment = get_next_segment(current_segment);
  
  return block;
}

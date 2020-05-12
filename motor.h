#pragma once
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#include <Arduino.h>

#include "macros.h"
#include "tmc2130.h"


// use in for(ALL_MOTORS(i)) { //i will be rising
#define ALL_MOTORS(NN) int NN=0;NN<NUM_MOTORS;++NN


//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------

#ifdef HAS_TMC2130
#else
// A4988
#define STEPPER_DIR_HIGH   HIGH
#define STEPPER_DIR_LOW    LOW
#endif


//------------------------------------------------------------------------------
// STRUCTURES
//------------------------------------------------------------------------------

typedef struct {
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
} SegmentAxis;


typedef struct {
  SegmentAxis a[NUM_MOTORS+NUM_SERVOS];

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
extern Motor motors[NUM_MOTORS+NUM_SERVOS];
extern const char *AxisNames;
extern const char *MotorNames;
extern float max_jerk[NUM_MOTORS+NUM_SERVOS];
extern float max_feedrate_mm_s[NUM_MOTORS+NUM_SERVOS];

extern uint8_t positionErrorFlags;
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

extern const int movesPlanned();

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

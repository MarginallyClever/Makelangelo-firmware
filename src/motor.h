#pragma once
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#include <Arduino.h>

#include "macros.h"
#include "tmc2130.h"

#if NUM_SERVOS > 0
#ifdef USE_ALT_SERVO
#  include "mservo.h"
#else
#  include <Servo.h>
#endif
#endif

//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------

#ifdef HAS_TMC2130
#else
// A4988
#  define STEPPER_DIR_HIGH HIGH
#  define STEPPER_DIR_LOW  LOW
#endif

#define NUM_MUSCLES (NUM_MOTORS + NUM_SERVOS)

//------------------------------------------------------------------------------
// MOVE BUFFERING
//------------------------------------------------------------------------------

// buffering commands
#ifndef MAX_SEGMENTS
#  define MAX_SEGMENTS (32)  // number of line segments to buffer ahead. must be a power of two.
#endif

#define SEGMOD(x) ((x) & (MAX_SEGMENTS - 1))

//------------------------------------------------------------------------------
// enable pin.  reverse if your board or drivers are odd.
//------------------------------------------------------------------------------

#ifndef MOTOR_ENABLE_ON
#define MOTOR_ENABLE_ON  LOW
#endif

#ifndef MOTOR_ENABLE_OFF
#define MOTOR_ENABLE_OFF  HIGH
#endif

//------------------------------------------------------------------------------
// step direction
//------------------------------------------------------------------------------

#ifndef START0
#  define START0 LOW
#endif
#ifndef START1
#  define START1 LOW
#endif
#ifndef START2
#  define START2 LOW
#endif
#ifndef START3
#  define START3 LOW
#endif
#ifndef START4
#  define START4 LOW
#endif
#ifndef START5
#  define START5 LOW
#endif

#ifndef END0
#  define END0 HIGH
#endif
#ifndef END1
#  define END1 HIGH
#endif
#ifndef END2
#  define END2 HIGH
#endif
#ifndef END3
#  define END3 HIGH
#endif
#ifndef END4
#  define END4 HIGH
#endif
#ifndef END5
#  define END5 HIGH
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
  int32_t step_count;   // current motor position, in steps.
  int32_t delta_steps;  // steps
  int32_t delta_units;     // in some systems it's mm, in others it's degrees.
  uint32_t absdelta;
  int dir;
#if MACHINE_STYLE == SIXI
  float expectedPosition;
  float positionStart;
  float positionEnd;
#endif
} Muscle;


enum BlockFlagBits : uint8_t { 
    BIT_FLAG_NOMINAL,
    BIT_FLAG_RECALCULATE
};

enum BlockFlagMask : uint8_t {
  BLOCK_FLAG_NOMINAL     = _BV(BIT_FLAG_NOMINAL),
  BLOCK_FLAG_RECALCULATE = _BV(BIT_FLAG_RECALCULATE),
};


typedef struct {
  Muscle a[NUM_MUSCLES];

  float distance;         // units
  float nominal_speed;    // units/s
  float entry_speed;      // units/s
  float entry_speed_max;  // units/s
  float acceleration;     // units/sec^2

  uint32_t steps_total;  // steps
  uint32_t steps_taken;  // steps
  uint32_t accel_until;  // steps
  uint32_t decel_after;  // steps

  uint32_t nominal_rate;               // steps/s
  uint32_t initial_rate;               // steps/s
  uint32_t final_rate;                 // steps/s
  uint32_t acceleration_steps_per_s2;  // steps/s^2
  uint32_t acceleration_rate;  // ?

  uint8_t flags;
} Segment;

//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------

extern Segment blockBuffer[MAX_SEGMENTS];
extern Segment *working_block;
extern volatile int block_buffer_head, block_buffer_nonbusy, block_buffer_planned, block_buffer_tail;
extern int first_segment_delay;
extern Motor motors[NUM_MUSCLES];
extern const char *AxisNames;

// max jerk value per axis
extern float max_jerk[NUM_MUSCLES];

// maximum feedrate (units/s).  one value per motor/servo 
extern float max_feedrate_units_s[NUM_MUSCLES];

// motor steps-per-unit.  one value per motor/servo
extern float motor_spu[NUM_MUSCLES];

extern uint32_t min_segment_time_us;

#if NUM_SERVOS>0
#ifndef ESP8266
extern Servo servos[NUM_SERVOS];
#endif
#endif

//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------

extern void motor_set_step_count(long *a);
extern void wait_for_empty_segment_buffer();
extern char segment_buffer_full();
extern void addSegment(const float *const target_position, float fr_units_s, float millimeters);
extern void motor_engage();
extern void motor_home();
extern void motor_disengage();
extern void motor_setup();
extern void setPenAngle(int arg0);
extern void clockISRProfile();

extern const int movesPlanned();
extern void motor_onestep(int motor);

#ifdef DEBUG_STEPPING
extern void debug_stepping();
#endif  // DEBUG_STEPPING

FORCE_INLINE const int movesPlanned() {
  return SEGMOD(block_buffer_head - block_buffer_tail);
}

FORCE_INLINE const int movesPlannedNotBusy() {
  return SEGMOD(block_buffer_head - block_buffer_nonbusy);
}

FORCE_INLINE const int getNextBlock(int i) {
  return SEGMOD(i + 1);
}

FORCE_INLINE const int getPrevBlock(int i) {
  return SEGMOD(i - 1);
}

FORCE_INLINE const bool isBlockBusy(const Segment *block) {
  return block == working_block;
}

FORCE_INLINE Segment *getCurrentBlock() {
  if (block_buffer_tail == block_buffer_head) return NULL;
  if (first_segment_delay > 0) {
    --first_segment_delay;
    if (movesPlanned() > 3) first_segment_delay = 0;
    return NULL;
  }

  Segment *block = &blockBuffer[block_buffer_tail];

  if( TEST(block->flags,BIT_FLAG_RECALCULATE) ) return NULL;  // wait, not ready

  block_buffer_nonbusy = getNextBlock(block_buffer_tail);
  if (block_buffer_tail == block_buffer_planned) {
    block_buffer_planned = block_buffer_nonbusy;
  }

  return block;
}

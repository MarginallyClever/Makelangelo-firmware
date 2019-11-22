#ifndef MOTOR_H
#define MOTOR_H
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#include <Arduino.h>
#ifdef HAS_TMC2130
#include <TMC2130Stepper.h>
#include <TMC2130Stepper_REGDEFS.h>

#if TMC2130STEPPER_VERSION < 0x020201
  #error "Update TMC2130Stepper library to 2.2.1 or newer."
#endif
#endif

//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------
#if MACHINE_STYLE == SIXI

#define POSITION_ERROR_FLAG_CONTINUOUS   (1<<0)  // report position (d17) continuously?
#define POSITION_ERROR_FLAG_ERROR        (1<<1)  // has error occurred?
#define POSITION_ERROR_FLAG_FIRSTERROR   (1<<2)  // report the error once per occurrence
#define POSITION_ERROR_FLAG_ESTOP        (1<<3)  // check for error at all?

#endif

#ifdef HAS_TMC2130
#define STEPPER_DIR_HIGH   LOW
#define STEPPER_DIR_LOW    HIGH

#define HOMING_OCR1A 776
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
  float distancePerStep;
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
extern volatile int current_segment;
extern volatile int last_segment;
extern int first_segment_delay;
extern Motor motors[NUM_MOTORS+NUM_SERVOS];
extern const char *AxisNames;
extern const char *MotorNames;
extern float max_jerk[NUM_MOTORS+NUM_SERVOS];
extern float max_feedrate_mm_s[NUM_MOTORS+NUM_SERVOS];

extern uint8_t positionErrorFlags;
extern uint32_t min_segment_time_us;


#ifdef HAS_TMC2130
extern bool homing;
extern TMC2130Stepper driver_0;
extern TMC2130Stepper driver_1;
#endif

extern void motor_set_step_count(long *a);
extern void wait_for_empty_segment_buffer();
extern char segment_buffer_full();
extern void motor_line(const float * const target_position,float &fr_mm_s);
extern void motor_engage();
extern void motor_home();
extern void enable_stealthChop();
extern void motor_disengage();
extern void motor_setup();
extern void setPenAngle(int arg0);

extern const int movesPlanned();

//extern FORCE_INLINE Segment *get_current_segment();
// for reasons I don't understand... if i put this method in the .h file i get compile errors.
// so I put it here, which forces the externs.
FORCE_INLINE Segment *get_current_segment() {
  if (current_segment == last_segment ) return NULL;
  if (first_segment_delay > 0) {
    --first_segment_delay;
    if (movesPlanned() > 3) first_segment_delay = 0;
    return NULL;
  }
  return &line_segments[current_segment];
}

#endif // MOTOR_H

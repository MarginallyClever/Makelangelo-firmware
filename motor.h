#ifndef MOTOR_H
#define MOTOR_H
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#include <Arduino.h>

//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------


typedef struct {
  int step_pin;
  int dir_pin;
  int enable_pin;
  int limit_switch_pin;
} Motor;


// for line()
typedef struct {
  long step_count;  // current motor position, in steps.
  long delta;  // steps
  long delta_mm;  // mm
  long absdelta;
  int dir;
} SegmentAxis;


typedef struct {
  SegmentAxis a[NUM_MOTORS+NUM_SERVOS];

  float distance;         // mm
  float nominal_speed;    // mm/s
  float entry_speed;      // mm/s
  float entry_speed_max;  // mm/s
  float acceleration;     // mm/sec^2

  int32_t steps_total;    // steps
  int32_t steps_taken;    // steps
  int32_t accel_until;    // steps
  int32_t decel_after;    // steps
  
  int32_t nominal_rate;   // steps/s
  int32_t entry_rate;     // steps/s
  int32_t exit_rate;      // steps/s
  int32_t acceleration_steps_per_s2;  // steps/s^2
  
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
extern Motor motors[NUM_MOTORS+NUM_SERVOS];
extern const char *AxisNames;
extern const char *MotorNames;
extern float maxFeedRate[NUM_MOTORS];

extern void motor_set_step_count(long *a);
extern void wait_for_empty_segment_buffer();
extern char segment_buffer_full();
extern void motor_line(const float * const target_position,float &fr_mm_s);
extern void motor_engage();
extern void motor_disengage();
extern void motor_setup();
extern void setPenAngle(int arg0);

#endif // MOTOR_H

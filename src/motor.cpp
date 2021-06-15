//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#include "configure.h"
#include "motor.h"

#include "lcd.h"
#include "speed_lookuptable.h"

//------------------------------------------------------------------------------
// DEFINES
//------------------------------------------------------------------------------

#ifdef CPU_32_BIT
  #define STEP_MULTIPLY(A,B) MultiU32X24toH32(A, B)
#else
  #define STEP_MULTIPLY(A,B) MultiU24X32toH16(A, B)
#endif

#define BLOCK_DELAY_FOR_1ST_MOVE 100
#define MIN_STEP_RATE            120
#define GRAVITYmag               (980.0)  // mm

//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------

Motor motors[NUM_MUSCLES];

#if NUM_SERVOS>0
#ifndef ESP8266
Servo servos[NUM_SERVOS];
#endif
#endif

Segment blockBuffer[MAX_SEGMENTS];
Segment *working_block         = NULL;
volatile int block_buffer_head = 0;
volatile int block_buffer_planned = 0;
volatile int block_buffer_nonbusy = 0;
volatile int block_buffer_tail = 0;
int first_segment_delay;

// used by timer1 to optimize interrupt inner loop
int steps_total;
int steps_taken;
int accel_until, decel_after;
uint32_t current_feed_rate;
uint32_t current_acceleration;
uint32_t start_feed_rate, end_feed_rate;
int32_t isr_nominal_rate = -1;
uint32_t time_accelerating, time_decelerating;
float max_jerk[NUM_MUSCLES];
float max_feedrate_units_s[NUM_MUSCLES];
float motor_spu[NUM_MUSCLES];
uint8_t isr_step_multiplier  = 1;
uint32_t min_segment_time_us = MIN_SEGMENT_TIME_US;

#define DECL_MOT(NN)      \
  int delta##NN;          \
  int over##NN;           \
  long global_steps_##NN; \
  int global_step_dir_##NN;

DECL_MOT(0)

#if NUM_MOTORS > 1
DECL_MOT(1)
#endif
#if NUM_MOTORS > 2
DECL_MOT(2)
#endif
#if NUM_MOTORS > 3
DECL_MOT(3)
#endif
#if NUM_MOTORS > 4
DECL_MOT(4)
#endif
#if NUM_MOTORS > 5
DECL_MOT(5)
#endif
#if NUM_SERVOS > 0
int servoDelta0;
int servoOver0;
long global_servoSteps_0;
int global_servoStep_dir_0;
#endif

float previous_nominal_speed = 0;
float previous_safe_speed    = 0;
float previous_speed[NUM_MUSCLES];

const char *AxisNames  = "XYZUVWT";

//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------

#ifdef CPU_32_BIT
static FORCE_INLINE uint32_t MultiU32X24toH32(uint32_t longIn1, uint32_t longIn2) {
  return ((uint64_t)longIn1 * longIn2 + 0x00800000) >> 24;
}

#else

#ifdef ESP8266
void itr();
#endif

// intRes = intIn1 * intIn2 >> 16
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 24 bit result
static FORCE_INLINE uint16_t MultiU16X8toH16(uint8_t charIn1, uint16_t intIn2) {
  register uint8_t tmp;
  register uint16_t intRes;
  __asm__ __volatile__(
    A("clr %[tmp]")
    A("mul %[charIn1], %B[intIn2]") 
    A("movw %A[intRes], r0")
    A("mul %[charIn1], %A[intIn2]") 
    A("add %A[intRes], r1") 
    A("adc %B[intRes], %[tmp]")
    A("lsr r0") 
    A("adc %A[intRes], %[tmp]") 
    A("adc %B[intRes], %[tmp]") 
    A("clr r1")
    : [ intRes ] "=&r"(intRes), [ tmp ] "=&r"(tmp)
    : [ charIn1 ] "d"(charIn1), [ intIn2 ] "d"(intIn2)
    : "cc"
  );
  return intRes;
}

// intRes = longIn1 * longIn2 >> 24
// uses:
// A[tmp] to store 0
// B[tmp] to store bits 16-23 of the 48bit result. The top bit is used to round the two byte result.
// note that the lower two bytes and the upper byte of the 48bit result are not calculated.
// this can cause the result to be out by one as the lower bytes may cause carries into the upper ones.
// B A are bits 24-39 and are the returned value
// C B A is longIn1
// D C B A is longIn2
//
static FORCE_INLINE uint16_t MultiU24X32toH16(uint32_t longIn1, uint32_t longIn2) {
#ifdef ESP8266
  uint16_t intRes = longIn1 * longIn2 >> 24;
#else   // ESP8266
  register uint8_t tmp1;
  register uint8_t tmp2;
  register uint16_t intRes;
  __asm__ __volatile__(
    A("clr %[tmp1]") 
    A("mul %A[longIn1], %B[longIn2]") 
    A("mov %[tmp2], r1") 
    A("mul %B[longIn1], %C[longIn2]")
    A("movw %A[intRes], r0") 
    A("mul %C[longIn1], %C[longIn2]") 
    A("add %B[intRes], r0")
    A("mul %C[longIn1], %B[longIn2]") 
    A("add %A[intRes], r0") 
    A("adc %B[intRes], r1")
    A("mul %A[longIn1], %C[longIn2]") 
    A("add %[tmp2], r0") 
    A("adc %A[intRes], r1")
    A("adc %B[intRes], %[tmp1]") 
    A("mul %B[longIn1], %B[longIn2]") 
    A("add %[tmp2], r0")
    A("adc %A[intRes], r1") 
    A("adc %B[intRes], %[tmp1]") 
    A("mul %C[longIn1], %A[longIn2]")
    A("add %[tmp2], r0") 
    A("adc %A[intRes], r1") 
    A("adc %B[intRes], %[tmp1]")
    A("mul %B[longIn1], %A[longIn2]") 
    A("add %[tmp2], r1") 
    A("adc %A[intRes], %[tmp1]")
    A("adc %B[intRes], %[tmp1]") 
    A("lsr %[tmp2]") 
    A("adc %A[intRes], %[tmp1]")
    A("adc %B[intRes], %[tmp1]") 
    A("mul %D[longIn2], %A[longIn1]")
    A("add %A[intRes], r0")
    A("adc %B[intRes], r1")
    A("mul %D[longIn2], %B[longIn1]") 
    A("add %B[intRes], r0") 
    A("clr r1")
    : [ intRes ] "=&r"(intRes), [ tmp1 ] "=&r"(tmp1), [ tmp2 ] "=&r"(tmp2)
    : [ longIn1 ] "d"(longIn1), [ longIn2 ] "d"(longIn2)
    : "cc"
  );
#endif  // ESP8266
  return intRes;
}
#endif //__AVR__

/**
   Calculate the maximum allowable speed at this point, in order
   to reach 'target_velocity' using 'acceleration' within a given
   'distance'.
   @param acc acceleration
   @param target_velocity
   @param distance
*/
float max_speed_allowed(const float &acc, const float &target_velocity, const float &distance) {
  return sqrt(sq(target_velocity) - 2 * acc * distance);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * set up the pins for each motor
 */
void motor_setup() {
#define SETUP_MOT(NN)                                    \
  motors[NN].letter           = MOTOR_##NN##_LETTER;     \
  motors[NN].step_pin         = MOTOR_##NN##_STEP_PIN;   \
  motors[NN].dir_pin          = MOTOR_##NN##_DIR_PIN;    \
  motors[NN].enable_pin       = MOTOR_##NN##_ENABLE_PIN; \
  motors[NN].limit_switch_pin = MOTOR_##NN##_LIMIT_SWITCH_PIN;

  SETUP_MOT(0)
#if NUM_MOTORS > 1
  SETUP_MOT(1)
#endif
#if NUM_MOTORS > 2
  SETUP_MOT(2)
#endif
#if NUM_MOTORS > 3
  SETUP_MOT(3)
#endif
#if NUM_MOTORS > 4
  SETUP_MOT(4)
#endif
#if NUM_MOTORS > 5
  SETUP_MOT(5)
#endif

  for(ALL_MOTORS(i)) {
    // set the motor pin & scale
    pinMode(motors[i].step_pin, OUTPUT);
    pinMode(motors[i].dir_pin, OUTPUT);
    pinMode(motors[i].enable_pin, OUTPUT);
    // set the switch pin
    pinMode(motors[i].limit_switch_pin, INPUT);
    digitalWrite(motors[i].limit_switch_pin, HIGH);

#ifdef HAS_TMC2130
    digitalWrite(motors[i].enable_pin, HIGH);  // deactivate driver (LOW active)
    digitalWrite(motors[i].step_pin, LOW);
#endif
  }

#ifdef HAS_TMC2130
  tmc2130_setup_all();
#endif

  for (ALL_MUSCLES(i)) {
    max_jerk[i]          = MAX_JERK_DEFAULT;
    max_feedrate_units_s[i] = MAX_FEEDRATE;
  }

#if MACHINE_STYLE == POLARGRAPH
#  ifdef MAX_FEEDRATE_Z
  max_feedrate_units_s[NUM_MOTORS] = MAX_FEEDRATE_Z;
#  endif
  max_jerk[NUM_MOTORS] = MAX_JERK_Z_DEFAULT;
#endif

  // setup servos
#if NUM_SERVOS > 0
  motors[NUM_MOTORS].letter = 'T';
#  ifdef ESP8266
  pinMode(SERVO0_PIN, OUTPUT);
#  else
  servos[0].attach(SERVO0_PIN);
#  endif  // ESP8266
#endif

#if NUM_SERVOS > 1
  servos[1].attach(SERVO1_PIN);
#endif
#if NUM_SERVOS > 2
  servos[2].attach(SERVO2_PIN);
#endif
#if NUM_SERVOS > 3
  servos[3].attach(SERVO3_PIN);
#endif
#if NUM_SERVOS > 4
  servos[4].attach(SERVO4_PIN);
#endif

  block_buffer_tail  = 0;
  block_buffer_head     = 0;
  Segment &old_seg = blockBuffer[getPrevBlock(block_buffer_head)];
  for (ALL_MUSCLES(i)) { old_seg.a[i].step_count = 0; }

  long steps[NUM_MUSCLES];
  memset(steps, 0, (NUM_MUSCLES) * sizeof(long));
  motor_set_step_count(steps);

  working_block         = NULL;
  first_segment_delay = 0;

  HAL_timer_start(STEP_TIMER_NUM);
  motor_engage();
}


// turn on power to the motors (make them immobile)
void motor_engage() {
  for (ALL_MOTORS(i)) {
    digitalWrite(motors[i].enable_pin, MOTOR_ENABLE_ON);
  }
  /*
    #if MACHINE_STYLE == SIXI
      // DM320T drivers want high for enabled
      digitalWrite(motors[4].enable_pin,HIGH);
      digitalWrite(motors[5].enable_pin,HIGH);
    #endif
  */
}

// turn off power to the motors (make them move freely)
void motor_disengage() {
  for (ALL_MOTORS(i)) {
    digitalWrite(motors[i].enable_pin, MOTOR_ENABLE_OFF);
  }
  /*
  #if MACHINE_STYLE == SIXI
  // DM320T drivers want low for disabled
  digitalWrite(motors[4].enable_pin,LOW);
  digitalWrite(motors[5].enable_pin,LOW);
  #endif
  */
}

// Change pen state.
void setPenAngle(int arg0) {
#if NUM_AXIES >= 3
  if (arg0 < axies[2].limitMin) arg0 = axies[2].limitMin;
  if (arg0 > axies[2].limitMax) arg0 = axies[2].limitMax;

  axies[2].pos = arg0;
#endif  // NUM_AXIES>=3

#if NUM_SERVOS > 0
#ifdef ESP8266
  analogWrite(SERVO0_PIN, arg0);
#else
  servos[0].write(arg0);
#endif  // ESP8266
#endif    // NUM_SERVOS>0
}

void recalculate_reverse_kernel(Segment *const current, const Segment *next) {
  if (current == NULL) return;

  const float entry_speed_max2 = current->entry_speed_max;
  if (current->entry_speed != entry_speed_max2 || (next && TEST(next->flags,BIT_FLAG_RECALCULATE)) ) {
    // If nominal length true, max junction speed is guaranteed to be reached. Only compute
    // for max allowable speed if block is decelerating and nominal length is false.
    const float new_entry_speed = TEST(current->flags, BIT_FLAG_NOMINAL)
                                  ? entry_speed_max2
                                  : min( entry_speed_max2, max_speed_allowed(-current->acceleration, (next ? next->entry_speed : MIN_FEEDRATE), current->distance) );

    if( current->entry_speed != new_entry_speed ) {
      SET_BIT_ON(current->flags, BIT_FLAG_RECALCULATE);
      if( isBlockBusy(current) ) {
        SET_BIT_OFF(current->flags, BIT_FLAG_RECALCULATE);
      } else {
        current->entry_speed = new_entry_speed;
      }
    }
  }
}

void recalculate_reverse() {
  int block_index = getPrevBlock(block_buffer_head);

  uint8_t planned_block_index = block_buffer_planned;
  if (planned_block_index == block_buffer_head) return;

  Segment *next = NULL;
  while (block_index != block_buffer_tail) {
    Segment *current = &blockBuffer[block_index];

    recalculate_reverse_kernel(current, next);
    next = current;
    block_index = getPrevBlock(block_index);
    while(planned_block_index != block_buffer_planned) {
      // If we reached the busy block or an already processed block, break the loop now
      if (block_index == planned_block_index) return;
      // Advance the pointer, following the busy block
      planned_block_index = getNextBlock(planned_block_index);
    }
  }
}

void recalculate_forward_kernel(const Segment *prev, Segment *const current,uint8_t block_index) {
  if (prev == NULL) return;

  // If the previous block is an acceleration block, but it is not long enough to complete the
  // full speed change within the block, we need to adjust the entry speed accordingly. Entry
  // speeds have already been reset, maximized, and reverse planned by reverse planner.
  // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
  if( !TEST(prev->flags,BIT_FLAG_NOMINAL) && prev->entry_speed < current->entry_speed ) {
    const float new_entry_speed2 = max_speed_allowed(-prev->acceleration, prev->entry_speed, prev->distance);
    // Check for junction speed change
    if (new_entry_speed2 < current->entry_speed) {
      SET_BIT_ON(current->flags, BIT_FLAG_RECALCULATE);
      if(isBlockBusy(current)) {
        SET_BIT_OFF(current->flags, BIT_FLAG_RECALCULATE);
      } else {
        current->entry_speed = new_entry_speed2;
        block_buffer_planned = block_index;
      }
    }
  }
}

void recalculate_forward() {
  int block_index = block_buffer_planned;

  Segment *previous = NULL;
  while (block_index != block_buffer_head) {
    Segment *current = &blockBuffer[block_index];
    if(!previous || !isBlockBusy(previous)) {
      recalculate_forward_kernel(previous, current,block_index);
    }
    previous = current;
    block_index = getNextBlock(block_index);
  }
}

float estimate_acceleration_distance(const float &initial_rate, const float &target_rate, const float &accel) {
  if (accel == 0) return 0;
  return (sq(target_rate) - sq(initial_rate)) / (accel * 2);
}

int intersection_distance(const float &start_rate, const float &end_rate, const float &accel, const float &distance) {
  if (accel == 0) return 0;
  return (2.0 * accel * distance - sq(start_rate) + sq(end_rate)) / (4.0 * accel);
}

void segment_update_trapezoid(Segment *s, const float &entry_factor, const float &exit_factor) {
  uint32_t intial_rate = ceil(s->nominal_rate * entry_factor);
  uint32_t final_rate  = ceil(s->nominal_rate * exit_factor);

  if (intial_rate < MIN_STEP_RATE) intial_rate = MIN_STEP_RATE;
  if (final_rate < MIN_STEP_RATE) final_rate = MIN_STEP_RATE;

  const int32_t accel       = s->acceleration_steps_per_s2;
  uint32_t accelerate_steps = ceil(estimate_acceleration_distance(intial_rate, s->nominal_rate, accel));
  uint32_t decelerate_steps = floor(estimate_acceleration_distance(s->nominal_rate, final_rate, -accel));
  int32_t plateau_steps     = s->steps_total - accelerate_steps - decelerate_steps;
  if (plateau_steps < 0) {
    const float accelerate_steps_float = ceil(intersection_distance(intial_rate, final_rate, accel, s->steps_total));
    accelerate_steps                   = min(((uint32_t)max(accelerate_steps_float, 0.0f)), s->steps_total);
    plateau_steps                      = 0;
  }
  s->accel_until  = accelerate_steps;
  s->decel_after  = accelerate_steps + plateau_steps;
  s->initial_rate = intial_rate;
  s->final_rate   = final_rate;
}

void recalculate_trapezoids() {
  uint8_t block_index = block_buffer_tail;
  uint8_t head_block_index = block_buffer_head;
  Segment *block = NULL;
  Segment *next    = NULL;

  float current_entry_speed = 0, next_entry_speed = 0;

  while (block_index != head_block_index) {
    next             = &blockBuffer[block_index];
    next_entry_speed = next->entry_speed;
    if(block) {
      // Recalculate if current block entry or exit junction speed has changed.
      if( TEST(block->flags,BIT_FLAG_RECALCULATE) || TEST(next->flags,BIT_FLAG_RECALCULATE) ) {
        SET_BIT_ON( block->flags, BIT_FLAG_RECALCULATE );
        if( !isBlockBusy(block) ) {
          // NOTE: Entry and exit factors always > 0 by all previous logic operations.
          const float inom = 1.0 / block->nominal_speed;
          segment_update_trapezoid(block, current_entry_speed * inom, next_entry_speed * inom);
        }
      }
      // Reset current only to ensure next trapezoid is computed
      SET_BIT_OFF(block->flags,BIT_FLAG_RECALCULATE);
    }
    block_index         = getNextBlock(block_index);
    current_entry_speed = next_entry_speed;
    block             = next;
  }

  // Last/newest block in buffer. Make sure the last block always ends motion.
  if (next) {
    SET_BIT_ON(next->flags,BIT_FLAG_RECALCULATE);
    if( !isBlockBusy(block) ) {
      const float inom = 1.0 / next->nominal_speed;
      segment_update_trapezoid(next, next_entry_speed * inom, MIN_FEEDRATE * inom);
    }
    SET_BIT_OFF(next->flags,BIT_FLAG_RECALCULATE);
  }
}

void describe_segments() {
  CRITICAL_SECTION_START();
  static uint8_t once = 0;
  if (once == 0) {
    once = 1;
    Serial.println("A = index");
    Serial.println("B = distance");
    Serial.println("C = acceleration");
    Serial.println("D = acceleration steps s2");
    Serial.println("E = acceleration rate");

    Serial.println("F = entry_speed");
    Serial.println("G = nominal_speed");
    Serial.println("H = entry_speed_max");

    Serial.println("I = entry rate");
    Serial.println("J = nominal rate");
    Serial.println("K = exit rate");

    Serial.println("L = accel_until");
    Serial.println("M = coast steps");
    Serial.println("N = decel steps");
    Serial.println("O = total steps");

    Serial.println("P = nominal?");
    Serial.println("Q = recalculate?");
    Serial.println("R = busy?"); /**/
    Serial.println("\nA\tB\tC\tD\tE\tF\tG\tH\tI\tJ\tK\tL\tM\tN\tO\tP\tQ\tR");
  }
  Serial.println("-----------------------------------------------------------------------------------------------------"
                 "----------------------");

  int s = block_buffer_tail;
  while (s != block_buffer_head) {
    Segment *next = &blockBuffer[s];
    int coast     = next->decel_after - next->accel_until;
    int decel     = next->steps_total - next->decel_after;
    Serial.print(s);
    Serial.print(F("\t"));   Serial.print(next->distance);
    Serial.print(F("\t"));   Serial.print(next->acceleration);
    Serial.print(F("\t"));   Serial.print(next->acceleration_steps_per_s2);
    Serial.print(F("\t"));   Serial.print(next->acceleration_rate);

    Serial.print(F("\t"));   Serial.print(next->entry_speed);
    Serial.print(F("\t"));   Serial.print(next->nominal_speed);
    Serial.print(F("\t"));   Serial.print(next->entry_speed_max);

    Serial.print(F("\t"));   Serial.print(next->initial_rate);
    Serial.print(F("\t"));   Serial.print(next->nominal_rate);
    Serial.print(F("\t"));   Serial.print(next->final_rate);

    Serial.print(F("\t"));   Serial.print(next->accel_until);
    Serial.print(F("\t"));   Serial.print(coast);
    Serial.print(F("\t"));   Serial.print(decel);
    Serial.print(F("\t"));   Serial.print(next->steps_total);
    //Serial.print(F("\t"));   Serial.print(next->steps_taken);

    Serial.print(F("\t"));   Serial.print(TEST(next->flags,BIT_FLAG_NOMINAL) != 0 ? 'Y' : 'N');
    Serial.print(F("\t"));   Serial.print(TEST(next->flags,BIT_FLAG_RECALCULATE) != 0 ? 'Y' : 'N');
    Serial.print(F("\t"));   Serial.print(isBlockBusy(next) != 0 ? 'Y' : 'N');
    Serial.println();
    s = getNextBlock(s);
  }
  CRITICAL_SECTION_END();
}

void recalculate_acceleration() {
  if(getPrevBlock(block_buffer_head) != block_buffer_planned) {
    recalculate_reverse();
    recalculate_forward();
  }
  recalculate_trapezoids();
  // describe_segments();
}

/**
 * Set the step count for each muscle.
 * @input a array of long values.  NUM_MUSCLES in length.
 */
void motor_set_step_count(long *a) {
  wait_for_empty_segment_buffer();

  previous_nominal_speed = 0;
  previous_safe_speed    = 0;
  for (ALL_MUSCLES(i)) { previous_speed[i] = 0; }

  Segment &old_seg = blockBuffer[getPrevBlock(block_buffer_head)];
  for (ALL_MUSCLES(i)) { old_seg.a[i].step_count = a[i]; }

  global_steps_0 = 0;
#if NUM_MOTORS > 1
  global_steps_1 = 0;
#endif
#if NUM_MOTORS > 2
  global_steps_2 = 0;
#endif
#if NUM_MOTORS > 3
  global_steps_3 = 0;
#endif
#if NUM_MOTORS > 4
  global_steps_4 = 0;
#endif
#if NUM_MOTORS > 5
  global_steps_5 = 0;
#endif
#if NUM_SERVOS > 0
  global_servoSteps_0 = 0;
#endif
}

/**
   Step one motor one time in the currently set direction.
   @input newx the destination x position
   @input newy the destination y position
 **/
void motor_onestep(int motor) {
#ifdef VERBOSE
  Serial.print(motors[motor].letter);
#endif

  digitalWrite(motors[motor].step_pin, HIGH);
  digitalWrite(motors[motor].step_pin, LOW);
}


#ifdef CPU_32_BIT
#define ISR_BASE_CYCLES                 800UL//752UL, was 1200, 800 too low
#define ISR_LOOP_BASE_CYCLES            4UL
#else
#define ISR_BASE_CYCLES                 1700UL//752UL, was 1200, 800 too low
#define ISR_LOOP_BASE_CYCLES            32UL
#endif
#define ISR_STEPPER_CYCLES              88UL

#define MIN_ISR_LOOP_CYCLES             (ISR_STEPPER_CYCLES * NUM_MUSCLES)
#define MAXIMUM_STEPPER_RATE            500000UL
#define MINIMUM_STEPPER_PULSE           1UL
  
#define _MIN_STEPPER_PULSE_CYCLES(N) max(  (F_CPU / MAXIMUM_STEPPER_RATE),  (F_CPU / 500000UL) * (N) )
  
#define MIN_STEPPER_PULSE_CYCLES       _MIN_STEPPER_PULSE_CYCLES(MINIMUM_STEPPER_PULSE)
#define ISR_LOOP_CYCLES                (ISR_LOOP_BASE_CYCLES + (long)max(MIN_STEPPER_PULSE_CYCLES, MIN_ISR_LOOP_CYCLES))
  
#define ISR_EXECUTION_CYCLES(R)  (  ( (ISR_BASE_CYCLES) + ((ISR_LOOP_CYCLES) * (R)) ) / (R) )

// The maximum allowable stepping frequency when doing x128-x1 stepping (in Hz)
#define MAX_STEP_ISR_FREQUENCY_128X ((F_CPU) / ISR_EXECUTION_CYCLES(128))
#define MAX_STEP_ISR_FREQUENCY_64X  ((F_CPU) / ISR_EXECUTION_CYCLES(64))
#define MAX_STEP_ISR_FREQUENCY_32X  ((F_CPU) / ISR_EXECUTION_CYCLES(32))
#define MAX_STEP_ISR_FREQUENCY_16X  ((F_CPU) / ISR_EXECUTION_CYCLES(16))
#define MAX_STEP_ISR_FREQUENCY_8X   ((F_CPU) / ISR_EXECUTION_CYCLES(8))
#define MAX_STEP_ISR_FREQUENCY_4X   ((F_CPU) / ISR_EXECUTION_CYCLES(4))
#define MAX_STEP_ISR_FREQUENCY_2X   ((F_CPU) / ISR_EXECUTION_CYCLES(2))
#define MAX_STEP_ISR_FREQUENCY_1X   ((F_CPU) / ISR_EXECUTION_CYCLES(1))

/**
   Set the clock 2 timer frequency.
   @input desired_freq_hz the desired frequency
*/
FORCE_INLINE static unsigned short calc_timer(uint32_t desired_freq_hz, uint8_t * loops) {
  uint32_t timer;
  uint8_t step_multiplier = 1;
  int idx=0;
  
  // The stepping frequency limits for each multistepping rate
  static const uint32_t limit[] PROGMEM = {
    (  MAX_STEP_ISR_FREQUENCY_1X     ),
    (  MAX_STEP_ISR_FREQUENCY_2X >> 1),
    (  MAX_STEP_ISR_FREQUENCY_4X >> 2),
    (  MAX_STEP_ISR_FREQUENCY_8X >> 3),
    ( MAX_STEP_ISR_FREQUENCY_16X >> 4),
    ( MAX_STEP_ISR_FREQUENCY_32X >> 5),
    ( MAX_STEP_ISR_FREQUENCY_64X >> 6),
    (MAX_STEP_ISR_FREQUENCY_128X >> 7)
  };

  while( idx<7 && desired_freq_hz > (uint32_t)pgm_read_dword(&limit[idx]) ) {
    step_multiplier <<= 1;
    desired_freq_hz >>= 1;
    idx++;
  }
  *loops = step_multiplier;


  #ifdef CPU_32_BIT
    timer = uint32_t(STEPPER_TIMER_RATE) / desired_freq_hz;
  #else
    if (desired_freq_hz < CLOCK_MIN_STEP_FREQUENCY) desired_freq_hz = CLOCK_MIN_STEP_FREQUENCY;
    desired_freq_hz -= CLOCK_MIN_STEP_FREQUENCY;
    if (desired_freq_hz >= 8 * 256) {
      const uint8_t tmp_step_rate  = (desired_freq_hz & 0x00FF);
      const uint16_t table_address = (uint16_t)&speed_lookuptable_fast[(uint8_t)(desired_freq_hz >> 8)][0],
                    gain          = (uint16_t)pgm_read_word_near(table_address + 2);
      timer                        = MultiU16X8toH16(tmp_step_rate, gain);
      timer                        = (uint16_t)pgm_read_word_near(table_address) - timer;
    } else {  // lower step rates
      uint16_t table_address = (uint16_t)&speed_lookuptable_slow[0][0];
      table_address += ((desired_freq_hz) >> 1) & 0xFFFC;
      timer = (uint16_t)pgm_read_word_near(table_address) -
              (((uint16_t)pgm_read_word_near(table_address + 2) * (uint8_t)(desired_freq_hz & 0x0007)) >> 3);
    }
  #endif

  return timer;
}

// Process pulsing in the isr step
inline void isr_internal_pulse() {
  if (working_block == NULL) return;

  //digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
  
  uint8_t i;

#if MACHINE_STYLE == SIXI
  if (TEST(sensorManager.positionErrorFlags, POSITION_ERROR_FLAG_ESTOP)) {
    // check if the sensor position differs from the estimated position.
    float fraction = (float)steps_taken / (float)steps_total;

    for (ALL_SENSORS(i)) {
      // interpolate live = (b-a)*f + a
      working_block->a[i].expectedPosition =
          (working_block->a[i].positionEnd - working_block->a[i].positionStart) * fraction +
          working_block->a[i].positionStart;

      float diff = abs(working_block->a[i].expectedPosition - sensorManager.sensors[i].angle);
      if (diff > POSITION_EPSILON) {
        // do nothing while the margin is too big.
        // Only end this condition is stopping the ISR, either via software disable or hardware reset.
        SET_BIT_ON(sensorManager.positionErrorFlags, POSITION_ERROR_FLAG_ERROR);
        return;
      }
    }
  }
#endif

  // move each axis
  for (i = 0; i < isr_step_multiplier; ++i) {
#ifdef DEBUG_STEPPING
    delayMicroseconds(150);
#endif
    over0 += delta0;
    if (over0 > 0) digitalWrite(MOTOR_0_STEP_PIN, START0);
#if NUM_MOTORS > 1
    over1 += delta1;
    if (over1 > 0) digitalWrite(MOTOR_1_STEP_PIN, START1);
#endif
#if NUM_MOTORS > 2
    over2 += delta2;
    if (over2 > 0) digitalWrite(MOTOR_2_STEP_PIN, START2);
#endif
#if NUM_MOTORS > 3
    over3 += delta3;
    if (over3 > 0) digitalWrite(MOTOR_3_STEP_PIN, START3);
#endif
#if NUM_MOTORS > 4
    over4 += delta4;
    if (over4 > 0) digitalWrite(MOTOR_4_STEP_PIN, START4);
#endif
#if NUM_MOTORS > 5
    over5 += delta5;
    if (over5 > 0) digitalWrite(MOTOR_5_STEP_PIN, START5);
#endif
#if NUM_SERVOS > 0
    servoOver0 += servoDelta0;
#endif
    // now that the pins have had a moment to settle, do the second half of the steps.
#define MOTOR_OVER(NN) \
    if(over##NN > 0) { \
      over##NN -= steps_total; \
      global_steps_##NN += global_step_dir_##NN; \
      digitalWrite(MOTOR_##NN##_STEP_PIN, END##NN); \
    }

    MOTOR_OVER(0);
#if NUM_MOTORS > 1
    MOTOR_OVER(1);
#endif
#if NUM_MOTORS > 2
    MOTOR_OVER(2);
#endif
#if NUM_MOTORS > 3
    MOTOR_OVER(3);
#endif
#if NUM_MOTORS > 4
    MOTOR_OVER(4);
#endif
#if NUM_MOTORS > 5
    MOTOR_OVER(5);
#endif
#if NUM_SERVOS > 0
    // servo 0
    if (servoOver0 > 0) {
      servoOver0 -= steps_total;
      global_servoSteps_0 += global_servoStep_dir_0;

#  ifdef ESP8266
      // analogWrite(SERVO0_PIN, global_servoSteps_0);
#  else
#    if !defined(HAS_GRIPPER)
      servos[0].write(global_servoSteps_0);
#    endif
#  endif
    }
#endif

    // make a step
    steps_taken++;
    if(steps_taken >= steps_total) break;
  }
}

// Process blocks in the isr
inline hal_timer_t isr_internal_block() {
  hal_timer_t interval = (HAL_TIMER_RATE) / 1000;

  if (working_block != NULL) {
    // Is this segment done?
    if (steps_taken >= steps_total) {
      // Move on to next segment without wasting an interrupt tick.
      working_block     = NULL;
      block_buffer_tail = getNextBlock(block_buffer_tail);
    } else {
      if (steps_taken <= accel_until) {
        // accelerating
        current_feed_rate = start_feed_rate + STEP_MULTIPLY(time_accelerating, current_acceleration);
        current_feed_rate = min(current_feed_rate, working_block->nominal_rate);
        interval          = calc_timer(current_feed_rate, &isr_step_multiplier);
        time_accelerating += interval;
#ifdef DEBUG_STEPPING
        /*
        Serial.print("A >> ");   Serial.print(interval);
        Serial.print("\t");      Serial.print(isr_step_multiplier);
        Serial.print("\t");      Serial.print(current_feed_rate);
        Serial.print(" = ");     Serial.print(start_feed_rate);
        Serial.print(" + ");     Serial.print(current_acceleration);
        Serial.print(" * ");     Serial.print(time_accelerating);
        Serial.println();//*/
#endif
      } else if (steps_taken > decel_after) {
        // decelerating
        hal_timer_t end_feed_rate = STEP_MULTIPLY(time_decelerating, current_acceleration);
        if (end_feed_rate < current_feed_rate) {
          end_feed_rate = current_feed_rate - end_feed_rate;
          end_feed_rate = max(end_feed_rate, working_block->final_rate);
        } else {
          end_feed_rate = working_block->final_rate;
        }
        interval = calc_timer(end_feed_rate, &isr_step_multiplier);
        time_decelerating += interval;
#ifdef DEBUG_STEPPING
        /*
        Serial.print("D >> ");   Serial.print(interval);
        Serial.print("\t");      Serial.print(isr_step_multiplier);
        Serial.print("\t");      Serial.print(end_feed_rate);
        Serial.print(" = ");     Serial.print(current_feed_rate);
        Serial.print(" - ");     Serial.print(current_acceleration);
        Serial.print(" * ");     Serial.print(time_decelerating);
        Serial.println();//*/
#endif
      } else {
        // cruising at nominal speed (flat top of the trapezoid)
        if (isr_nominal_rate < 0) { isr_nominal_rate = calc_timer(working_block->nominal_rate, &isr_step_multiplier); }
        interval = isr_nominal_rate;
#ifdef DEBUG_STEPPING
        /*
        Serial.print("N >> ");   Serial.println(interval);
        Serial.print("\t");      Serial.print(isr_step_multiplier);
        Serial.print(" / ");     Serial.print(working_block->nominal_rate);
        Serial.println();//*/
#endif
      }
    }
  }

  // segment buffer empty? do nothing
  if (working_block == NULL) {
    working_block = getCurrentBlock();

    if (working_block == NULL) return interval;  // buffer empty

    // set the direction pins
    digitalWrite(MOTOR_0_DIR_PIN, working_block->a[0].dir);
    global_step_dir_0 = (working_block->a[0].dir == STEPPER_DIR_HIGH) ? 1 : -1;

#if NUM_MOTORS > 1
    digitalWrite(MOTOR_1_DIR_PIN, working_block->a[1].dir);
    global_step_dir_1 = (working_block->a[1].dir == STEPPER_DIR_HIGH) ? 1 : -1;
#endif
#if NUM_MOTORS > 2
    digitalWrite(MOTOR_2_DIR_PIN, working_block->a[2].dir);
    global_step_dir_2 = (working_block->a[2].dir == STEPPER_DIR_HIGH) ? 1 : -1;
#endif
#if NUM_MOTORS > 3
    digitalWrite(MOTOR_3_DIR_PIN, working_block->a[3].dir);
    global_step_dir_3 = (working_block->a[3].dir == STEPPER_DIR_HIGH) ? 1 : -1;
#endif
#if NUM_MOTORS > 4
    digitalWrite(MOTOR_4_DIR_PIN, working_block->a[4].dir);
    global_step_dir_4 = (working_block->a[4].dir == STEPPER_DIR_HIGH) ? 1 : -1;
#endif
#if NUM_MOTORS > 5
    digitalWrite(MOTOR_5_DIR_PIN, working_block->a[5].dir);
    global_step_dir_5 = (working_block->a[5].dir == STEPPER_DIR_HIGH) ? 1 : -1;
#endif
#if NUM_SERVOS > 0
    global_servoStep_dir_0 = (working_block->a[NUM_MOTORS].dir == STEPPER_DIR_HIGH) ? -1 : 1;
#endif

    start_feed_rate      = working_block->initial_rate;
    end_feed_rate        = working_block->final_rate;
    current_feed_rate    = start_feed_rate;
    current_acceleration = working_block->acceleration_rate;
    accel_until          = working_block->accel_until;
    decel_after          = working_block->decel_after;
    time_accelerating    = 0;
    time_decelerating    = 0;
    isr_nominal_rate     = -1;

    interval = calc_timer(current_feed_rate, &isr_step_multiplier);

    // defererencing some data so the loop runs faster.
    steps_total = working_block->steps_total;
    steps_taken = 0;
    delta0      = working_block->a[0].absdelta;
    over0       = -(steps_total >> 1);
#if NUM_MOTORS > 1
    delta1 = working_block->a[1].absdelta;
    over1  = -(steps_total >> 1);
#endif
#if NUM_MOTORS > 2
    delta2 = working_block->a[2].absdelta;
    over2  = -(steps_total >> 1);
#endif
#if NUM_MOTORS > 3
    delta3 = working_block->a[3].absdelta;
    over3  = -(steps_total >> 1);
#endif
#if NUM_MOTORS > 4
    delta4 = working_block->a[4].absdelta;
    over4  = -(steps_total >> 1);
#endif
#if NUM_MOTORS > 5
    delta5 = working_block->a[5].absdelta;
    over5  = -(steps_total >> 1);
#endif
#if NUM_SERVOS > 0
    global_servoSteps_0 = working_block->a[NUM_MOTORS].step_count - working_block->a[NUM_MOTORS].delta_steps;
    servoDelta0 = working_block->a[NUM_MOTORS].absdelta;
    servoOver0  = -(steps_total >> 1);

#    if defined(HAS_GRIPPER)
      gripper.sendPositionRequest(working_block->a[NUM_MOTORS].step_count, 255, 32);
#    endif
#endif

#ifdef DEBUG_STEPPING
    int decel   = working_block->steps_total - working_block->decel_after;
    int nominal = working_block->decel_after - working_block->accel_until;
    Serial.print("seg: ");
    Serial.print((long)working_block, HEX);
    // Serial.print("  distance: ");  Serial.println(working_block->distance);
    // Serial.print("  nominal_speed: ");  Serial.println(working_block->nominal_speed);
    Serial.print("  entry_speed: ");
    Serial.print(working_block->entry_speed);
    Serial.print("  entry_speed_max: ");
    Serial.print(working_block->entry_speed_max);
    // Serial.print("  acceleration: ");  Serial.println(working_block->acceleration);
    Serial.print("  accel: ");
    Serial.print(working_block->accel_until);
    Serial.print("  nominal: ");
    Serial.print(nominal);
    Serial.print("  decel: ");
    Serial.println(decel);
    // Serial.print("  initial_rate: ");  Serial.println(working_block->initial_rate);
    // Serial.print("  nominal_rate: ");  Serial.println(working_block->nominal_rate);
    // Serial.print("  final_rate: ");  Serial.println(working_block->final_rate);
    // Serial.print("  acceleration_steps_per_s2: ");  Serial.println(working_block->acceleration_steps_per_s2);
    // Serial.print("  acceleration_rate: ");  Serial.println(working_block->acceleration_rate);
    // Serial.print("  nominal_length_flag: ");  Serial.println(working_block->nominal_length_flag==0?"n":"y");
    // Serial.print("  recalculate_flag: ");  Serial.println(working_block->recalculate_flag==0?"n":"y");
    // Serial.print("  dx: ");  Serial.println(working_block->a[0].delta_units);
    // Serial.print("  dy: ");  Serial.println(working_block->a[1].delta_units);
    // Serial.print("  dz: ");  Serial.println(working_block->a[2].delta_units);
#endif
  }

  return interval;
}

#ifdef DEBUG_STEPPING
void debug_stepping() {
  isr_internal_pulse();
  uint32_t interval = isr_internal_block();
  // Serial.println(interval);
}
#endif

HAL_STEP_TIMER_ISR {
  static hal_timer_t nextMainISR=0;
  
  //digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));

  #ifndef __AVR__
    // Disable interrupts, to avoid ISR preemption while we reprogram the period
    // (AVR enters the ISR with global interrupts disabled, so no need to do it here)
    DISABLE_ISRS();
  #endif
  
  // set the timer interrupt value as big as possible so there's little chance it triggers while i'm still in the ISR.
  HAL_timer_set_compare(STEP_TIMER_NUM, hal_timer_t(HAL_TIMER_TYPE_MAX));

  uint8_t max_loops       = 10;
  hal_timer_t next_isr_ticks = 0;
  hal_timer_t min_ticks;
  do {
    // Turn the interrupts back on (reduces UART delay, apparently)
    ENABLE_ISRS();

#ifndef DEBUG_STEPPING
#  ifdef HAS_TMC2130
    if (homing == true) {
      tmc2130_homing_sequence();
      nextMainISR = HOMING_OCR1A;
    } else {
#  endif
      if (!nextMainISR) isr_internal_pulse();
      if (!nextMainISR) nextMainISR = isr_internal_block();
#  ifdef HAS_TMC2130
    }
#  endif
#endif  // DEBUG_STEPPING

    hal_timer_t interval = min(hal_timer_t(HAL_TIMER_TYPE_MAX),nextMainISR);

    nextMainISR      -= interval;
    next_isr_ticks   += interval;

    DISABLE_ISRS();
    min_ticks = HAL_timer_get_count(STEP_TIMER_NUM) + hal_timer_t(
      #ifdef __AVR__
        8
      #else
        1
      #endif
      * (STEPPER_TIMER_TICKS_PER_US)
    );

    if (!--max_loops) next_isr_ticks = min_ticks;
    // ORC1A has been advancing while the interrupt was running.
    // if OCR1A is too close to the timer, do the step again immediately
  } while (next_isr_ticks < min_ticks);

  // set the next isr to fire at the right time.
  HAL_timer_set_compare(STEP_TIMER_NUM, hal_timer_t(next_isr_ticks));

  // turn the interrupts back on
  ENABLE_ISRS();
}

void clockISRProfile() {
  // Disable interrupts, to avoid ISR preemption while we reprogram the period
  // CRITICAL_SECTION_START();
  // make sure the isr_step_multiplier is 1
  int oldMult         = isr_step_multiplier;
  isr_step_multiplier = 1;

  int count = 1000;
  // get set... go!
  uint32_t tStart = micros();

  for (int i = 0; i < count; ++i) isr_internal_pulse();

  uint32_t tEnd = micros();

  // restore isr_step_multiplier
  isr_step_multiplier = oldMult;

  // Turn the interrupts back on (reduces UART delay, apparently)
  // CRITICAL_SECTION_END();

  // report results
  uint32_t dt = tEnd - tStart;
  float dtPer = (float)dt / (float)count;
  Serial.print(F("profile loops     ="));
  Serial.println(count);
  Serial.print(F("profile total time="));
  Serial.println(dt);
  Serial.print(F("profile per loop  ="));
  Serial.println(dtPer);
}

/**
   @return 1 if buffer is full, 0 if it is not.
*/
char segment_buffer_full() {
  int next_segment = getNextBlock(block_buffer_head);
  return (next_segment == block_buffer_tail);
}

FORCE_INLINE static Segment *getNextFreeBlock(uint8_t &next_buffer_head,const uint8_t count=1) {
  // get the next available spot in the segment buffer
  while (getNextBlock(block_buffer_head) == block_buffer_tail) {
    // the segment buffer is full, we are way ahead of the motion system.  wait here.
    meanwhile();
  }

  next_buffer_head = getNextBlock(block_buffer_head);
  return &blockBuffer[block_buffer_head];
}

void addSteps(Segment *newBlock,const float *const target_position, float fr_units_s, float longest_distance) {
  int prev_segment = getPrevBlock(block_buffer_head);
  Segment &oldBlock = blockBuffer[prev_segment];

  // convert from the cartesian position to the motor steps
  long steps[NUM_MUSCLES];
  IK(target_position, steps);

#if MACHINE_STYLE == POLARGRAPH
  float oldX = axies[0].pos;
  float oldY = axies[1].pos;
  // float oldZ = axies[2].pos;
#endif

  // find the number of steps for each motor, the direction, and the absolute steps
  // The axis that has the most steps will control the overall acceleration as per bresenham's algorithm.
  newBlock->steps_total = 0;

  for (ALL_MUSCLES(i)) {
    newBlock->a[i].step_count  = steps[i];
    newBlock->a[i].delta_steps = steps[i] - oldBlock.a[i].step_count;

    newBlock->a[i].dir = (newBlock->a[i].delta_steps < 0 ? STEPPER_DIR_HIGH : STEPPER_DIR_LOW);
#if MACHINE_STYLE == SIXI
    new_seg.a[i].positionStart = axies[i].pos;
    new_seg.a[i].positionEnd   = target_position[i];
#endif
    newBlock->a[i].delta_units = newBlock->a[i].delta_steps / motor_spu[i];
    newBlock->a[i].absdelta = abs(newBlock->a[i].delta_steps);
    if (newBlock->steps_total < newBlock->a[i].absdelta) newBlock->steps_total = newBlock->a[i].absdelta;
  }

  for (ALL_AXIES(i)) axies[i].pos = target_position[i];

  // No steps?  No work!  Stop now.
  if (newBlock->steps_total < MIN_STEPS_PER_SEGMENT) return;

  newBlock->distance = longest_distance;

  // record the new target position & feed rate for the next movement.
  float inverse_distance_units = 1.0 / longest_distance;
  float inverse_secs = fr_units_s * inverse_distance_units;
  int movesQueued = movesPlannedNotBusy();
  uint32_t segment_time_us = lroundf(1000000.0f / inverse_secs);

#ifdef BUFFER_EMPTY_SLOWDOWN
  if (movesQueued >= 2 && movesQueued <= (MAX_SEGMENTS / 2) - 1) {
    const int32_t time_diff = min_segment_time_us - segment_time_us;
    if(time_diff > 0) {
      // Serial.print("was ");  Serial.print(inverse_secs);
      const uint32_t nst = segment_time_us + lroundf(2 * time_diff  / movesQueued);
      inverse_secs       = 1000000.0f / nst;
      // Serial.print(" now ");  Serial.println(inverse_secs);
    }
  }
#endif

  newBlock->nominal_speed = longest_distance * inverse_secs;
  newBlock->nominal_rate  = ceil(newBlock->steps_total * inverse_secs);

  // Calculate the the speed limit for each axis.
  // All speeds are connected so if one motor slows, they all have to slow the same amount.
  float current_speed[NUM_MUSCLES], speed_factor = 1.0;
  for (ALL_MUSCLES(i)) {
    current_speed[i] = newBlock->a[i].delta_units * inverse_secs;
    const float cs = fabs(current_speed[i]), max_fr = max_feedrate_units_s[i];
    if (cs > max_fr) speed_factor = min(speed_factor, max_fr / cs);
  }
  // apply the speed limit
  if (speed_factor < 1.0) {
    for (ALL_MUSCLES(i)) {
      current_speed[i] *= speed_factor;
    }
    newBlock->nominal_speed *= speed_factor;
    newBlock->nominal_rate *= speed_factor;
  }

  // acceleration is a global set with "G0 A*"
  float max_acceleration = acceleration;

#if MACHINE_STYLE == POLARGRAPH && defined(DYNAMIC_ACCELERATION)
  {
    // Adjust the maximum acceleration based on the plotter position to reduce wobble at the bottom of the picture.
    // We only consider the XY plane.
    // Special thanks to https://www.reddit.com/user/zebediah49 for his math help.

    // if T is your target direction unit vector,
    float Tx = target_position[0] - oldX;
    float Ty = target_position[1] - oldY;
    float Rlen = sq(Tx) + sq(Ty);  // always >=0
    if (Rlen > 0) {
      // normal vectors pointing from plotter to motor
      float R1x   = axies[0].limitMin - oldX;  // to left
      float R1y   = axies[1].limitMax - oldY;  // to top
      float Rlen1 = 1.0 / sqrt(sq(R1x) + sq(R1y));//old_seg.a[0].step_count * UNITS_PER_STEP;
      R1x *= Rlen1;
      R1y *= Rlen1;

      float R2x   = axies[0].limitMax - oldX;  // to right
      float R2y   = axies[1].limitMax - oldY;  // to top
      float Rlen2 = 1.0 / sqrt(sq(R2x) + sq(R2y));//old_seg.a[1].step_count * UNITS_PER_STEP;
      R2x *= Rlen2;
      R2y *= Rlen2;
      
      // only affects XY non-zero movement.  Servo is not touched.
      Rlen = 1.0 / sqrt(Rlen);
      Tx *= Rlen;
      Ty *= Rlen;

      // solve cT = -gY + k1 R1 for c [and k1]
      // solve cT = -gY + k2 R2 for c [and k2]
      float c1 = -GRAVITYmag * R1x / (Tx * R1y - Ty * R1x);
      float c2 = -GRAVITYmag * R2x / (Tx * R2y - Ty * R2x);

      // If c is negative, that means that that support rope doesn't limit the acceleration; discard that c.
      float cT = -1;
      if (c1 > 0 && c2 > 0) {
        cT = (c1 < c2) ? c1 : c2;
      } else if (c1 > 0) {
        cT = c1;
      } else if (c2 > 0) {
        cT = c2;
      }

      // The maximum acceleration is given by cT if cT>0
      if (cT > 0) {
        max_acceleration = max(min(max_acceleration, cT), (float)MIN_ACCELERATION);
      }
    }
  }
#endif  // MACHINE_STYLE == POLARGRAPH && defined(DYNAMIC_ACCELERATION)

  const float steps_per_unit = newBlock->steps_total * inverse_distance_units;
  uint32_t accel           = ceil(max_acceleration * steps_per_unit);

  uint32_t highest_rate = 1;
  float max_acceleration_steps_per_s2[NUM_MUSCLES];

  for (ALL_MUSCLES(i)) {
    max_acceleration_steps_per_s2[i] = max_acceleration * motor_spu[i];
    highest_rate = max( highest_rate, max_acceleration_steps_per_s2[i] );
  }
  uint32_t acceleration_long_cutoff = 4294967295UL / highest_rate; // 0xFFFFFFFFUL

  if(newBlock->steps_total <= acceleration_long_cutoff) {
    for (ALL_MUSCLES(i)) {
      if (newBlock->a[i].absdelta && max_acceleration_steps_per_s2[i] < accel) {
        const uint32_t max_possible = max_acceleration_steps_per_s2[i] * newBlock->steps_total / newBlock->a[i].absdelta;
        accel = min( accel, max_possible );
      }
    }
  } else {
    for (ALL_MUSCLES(i)) {
      if (newBlock->a[i].absdelta && max_acceleration_steps_per_s2[i] < accel) {
        const float max_possible = float(max_acceleration_steps_per_s2[i]) * float(newBlock->steps_total) / float(newBlock->a[i].absdelta);
        accel = min( accel, max_possible );
      }
    }
  }

  newBlock->acceleration_steps_per_s2 = accel;
  newBlock->acceleration              = accel / steps_per_unit;
  newBlock->acceleration_rate         = (uint32_t)(accel * (sq(4096.0f) / (STEPPER_TIMER_RATE)));
  newBlock->steps_taken               = 0;

  // BEGIN JERK LIMITING

  float safe_speed = newBlock->nominal_speed;
  char limited     = 0;
  for (ALL_MUSCLES(i)) {
    const float jerk = fabs(current_speed[i]), maxj = max_jerk[i];
    if (jerk > maxj) {  // new current speed too fast?
      if (limited) {
        const float mjerk = maxj * newBlock->nominal_speed;          // ns*mj
        if (jerk * safe_speed > mjerk) safe_speed = mjerk / jerk;  // ns*mj/cs
      } else {
        safe_speed *= maxj / jerk;  // Initial limit: ns*mj/cs
        ++limited;
      }
    }
  }

  // what is the maximum starting speed for this segment?
  float vmax_junction;
  /*
  #if MACHINE_STYLE == POLARGRAPH
    if(new_seg.a[2].absdelta != old_seg.a[2].absdelta) {    // any polargraph change in z (servo horn action) must begin
  and end with a full stop.
    //if(new_seg.a[2].absdelta == PEN_DOWN_ANGLE && old_seg.a[2].absdelta == PEN_UP_ANGLE ) {    // any polargraph
  lowering (servo horn action) must begin with a full stop. vmax_junction=0; } else #endif*/
  if (movesQueued > 0 && previous_nominal_speed > 1e-6) {
    // Estimate a maximum velocity allowed at a joint of two successive segments.
    // If this maximum velocity allowed is lower than the minimum of the entry / exit safe velocities,
    // then the machine is not coasting anymore and the safe entry / exit velocities shall be used.

    // The junction velocity will be shared between successive segments. Limit the junction velocity to their minimum.
    // Pick the smaller of the nominal speeds. Higher speed shall not be achieved at the junction during coasting.
    vmax_junction = min(newBlock->nominal_speed, previous_nominal_speed);

    // Factor to multiply the previous / current nominal velocities to get componentwise limited velocities.
    float v_factor = 1.0f;
    limited        = 0;

    // Now limit the jerk in all axes.
    const float smaller_speed_factor = vmax_junction / previous_nominal_speed;
    for (ALL_MUSCLES(i)) {
      // Limit an axis. We have to differentiate: coasting, reversal of an axis, full stop.
      float v_exit  = previous_speed[i] * smaller_speed_factor;
      float v_entry = current_speed[i];
      if (limited) {
        v_exit *= v_factor;
        v_entry *= v_factor;
      }

      // Calculate jerk depending on whether the axis is coasting in the same direction or reversing.
      const float jerk = (v_exit > v_entry) ?  //                            coasting             axis reversal
                             ((v_entry > 0 || v_exit < 0) ? (v_exit - v_entry) : max(v_exit, -v_entry))
                                            :  // v_exit <= v_entry          coasting             axis reversal
                             ((v_entry < 0 || v_exit > 0) ? (v_entry - v_exit) : max(-v_exit, v_entry));

      if (jerk > max_jerk[i]) {
        v_factor *= max_jerk[i] / jerk;
        ++limited;
      }
    }

    if (limited) { vmax_junction *= v_factor; }
    // Now the transition velocity is known, which maximizes the shared exit / entry velocity while
    // respecting the jerk factors, it may be possible, that applying separate safe exit / entry velocities will achieve
    // faster prints.
    const float vmax_junction_threshold = vmax_junction * 0.99f;
    if (previous_safe_speed > vmax_junction_threshold && safe_speed > vmax_junction_threshold) {
      // Not coasting. The machine will stop and start the movements anyway,
      // better to start the segment from start.
      vmax_junction = safe_speed;
    }
  } else {
    vmax_junction = safe_speed;
  }

  // END JERK LIMITING

  previous_safe_speed = safe_speed;

  float allowable_speed = max_speed_allowed(-newBlock->acceleration, MIN_FEEDRATE, newBlock->distance);

  newBlock->entry_speed_max = vmax_junction;
  newBlock->entry_speed     = min(vmax_junction, allowable_speed);
  SET_BIT( newBlock->flags, BIT_FLAG_NOMINAL, ( allowable_speed >= newBlock->nominal_speed ) );
  SET_BIT_ON( newBlock->flags, BIT_FLAG_RECALCULATE );

  previous_nominal_speed = newBlock->nominal_speed;
  for(ALL_MUSCLES(i)) {
    previous_speed[i] = current_speed[i];
  }

  // when should we accelerate and decelerate in this segment?
  segment_update_trapezoid(newBlock, newBlock->entry_speed / newBlock->nominal_speed,
                           (float)MIN_FEEDRATE / newBlock->nominal_speed);
  /*
    Serial.print("seg:");  Serial.println((long)&new_seg,HEX);
    Serial.print("distance=");  Serial.println(new_seg.distance);
    Serial.print("acceleration original=");  Serial.println(acceleration);
    Serial.print("nominal_speed=");  Serial.println(new_seg.nominal_speed);

    Serial.print("inverse_distance_units=");  Serial.println(inverse_distance_units);
    Serial.print("inverse_secs=");  Serial.println(inverse_secs);
    Serial.print("nominal_rate=");  Serial.println(new_seg.nominal_rate);
    Serial.print("delta_units=");
    for (ALL_MUSCLES(i)) {
      if (i > 0) Serial.print(",");
      Serial.print(new_seg.a[i].delta_units);
    }
    Serial.println();
    Serial.print("speed_factor=");  Serial.println(speed_factor);
    Serial.print("steps_per_unit=");  Serial.println(steps_per_unit);
    Serial.print("accel=");  Serial.println(accel);
    Serial.print("acceleration_steps_per_s2=");  Serial.println(new_seg.acceleration_steps_per_s2);
    Serial.print("acceleration=");  Serial.println(new_seg.acceleration);
    Serial.print("limited=");  Serial.println(limited, DEC);
    Serial.print("nominal_speed=");  Serial.println(new_seg.nominal_speed);
    Serial.print("vmax_junction=");  Serial.println(vmax_junction);
    Serial.print("allowable_speed=");  Serial.println(allowable_speed);
    Serial.print("safe_speed=");  Serial.println(safe_speed);
    Serial.print("entry_speed_max=");  Serial.println(new_seg.entry_speed_max);
    Serial.print("entry_speed=");  Serial.println(new_seg.entry_speed);
    //*/
}

/**
   Translate the XYZ through the IK to get the number of motor steps and move the motors.
   Uses bresenham's line algorithm to move both motors
   @input pos NUM_AXIES floats describing destination coordinates
   @input new_feed_rate speed to travel along arc
   @input longest_distance must be >=0
*/
void addSegment(const float *const target_position, float fr_units_s, float longest_distance) {
  uint8_t next_buffer_head;
  Segment *newBlock = getNextFreeBlock(next_buffer_head);

  addSteps(newBlock,target_position,fr_units_s,longest_distance);

  if (block_buffer_tail == block_buffer_head) {
    first_segment_delay = BLOCK_DELAY_FOR_1ST_MOVE;
  }
  block_buffer_head = next_buffer_head;

  recalculate_acceleration();

  ENABLE_STEPPER_DRIVER_INTERRUPT();
}


void wait_for_empty_segment_buffer() {
  while (block_buffer_tail != block_buffer_head);
}

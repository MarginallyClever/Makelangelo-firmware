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
float max_step_rate_s[NUM_MUSCLES];
float motor_spu[NUM_MUSCLES];
uint8_t isr_step_multiplier  = 1;
uint32_t min_segment_time_us = DEFAULT_MIN_SEGMENT_TIME_US;

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

  block_buffer_tail = 0;
  block_buffer_head = 0;
  
  long steps[NUM_MUSCLES];
  memset(steps, 0, (NUM_MUSCLES) * sizeof(long));
  motor_set_step_count(steps);

  working_block = NULL;
  first_segment_delay = 0;

  HAL_timer_start(STEP_TIMER_NUM);
  motor_engage();
}


/**
 * Set the step count for each muscle.
 * @input a array of long values.  NUM_MUSCLES in length.
 */
void motor_set_step_count(long *a) {
  wait_for_empty_segment_buffer();
  planner_zeroSpeeds();

  Segment &old_seg = blockBuffer[getPrevBlock(block_buffer_head)];
  for (ALL_MUSCLES(i)) old_seg.a[i].step_count = a[i];

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
#define ISR_BASE_CYCLES                 900UL
#define ISR_LOOP_BASE_CYCLES            4UL
#else
#define ISR_BASE_CYCLES                 900UL
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

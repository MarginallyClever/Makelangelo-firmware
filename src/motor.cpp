//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#include "configure.h"

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

Stepper motor;

Motor motors[NUM_MUSCLES];


#if NUM_SERVOS>0
#ifndef ESP8266
Servo servos[NUM_SERVOS];
#endif
#endif

Segment *working_block = NULL;

// used by timer1 to optimize interrupt inner loop
int steps_total;
int steps_taken;
int accel_until, decel_after;
uint32_t acc_step_rate;
int32_t isr_nominal_rate = -1;
uint32_t time_accelerating, time_decelerating;
float max_jerk[NUM_MUSCLES];
float max_step_rate_s[NUM_MUSCLES];
float motor_spu[NUM_MUSCLES];
uint8_t isr_step_multiplier  = 1;
uint32_t min_segment_time_us = DEFAULT_MIN_SEGMENT_TIME_US;
uint16_t directionBits=0;
uint32_t advance_divisor;

#define DECL_MOT(NN)      \
  uint32_t delta##NN;          \
  int32_t over##NN;           \
  uint32_t global_steps_##NN; \
  int8_t global_step_dir_##NN;

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

#ifdef ESP8266
void itr();
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * set up the pins for each motor
 */
void Stepper::setup() {
#define SETUP_MOT(NN)                                    \
  motors[NN].letter           = MOTOR_##NN##_LETTER;     \
  motors[NN].step_pin         = MOTOR_##NN##_STEP_PIN;   \
  motors[NN].dir_pin          = MOTOR_##NN##_DIR_PIN;    \
  motors[NN].enable_pin       = MOTOR_##NN##_ENABLE_PIN; \
  motors[NN].limit_switch_pin = MOTOR_##NN##_LIMIT_SWITCH_PIN;

  ALL_MOTOR_MACRO(SETUP_MOT);

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

  int32_t steps[NUM_MUSCLES];
  memset(steps, 0, (NUM_MUSCLES) * sizeof(long));
  set_step_count(steps);

  setDirections(0);

  HAL_timer_start(STEP_TIMER_NUM);
  engage();
}


/**
 * Set the step count for each muscle.
 * @input NUM_MUSCLES in length.
 */
void Stepper::set_step_count(int32_t *a) {
  planner.zeroSpeeds();

  Segment &old_seg = planner.blockBuffer[planner.getPrevBlock(planner.block_buffer_head)];
  for (ALL_MUSCLES(i)) old_seg.a[i].step_count = a[i];

#define SETUP_STEP(NN) global_steps_##NN = 0;
  ALL_MOTOR_MACRO(SETUP_STEP);

#if NUM_SERVOS > 0
  global_servoSteps_0 = 0;
#endif
}


// turn on power to the motors (make them immobile)
void Stepper::engage() {
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
void Stepper::disengage() {
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
void Stepper::setPenAngle(int arg0) {
#if NUM_AXIES >= 3
  if(arg0 < axies[2].limitMin) arg0 = axies[2].limitMin;
  if(arg0 > axies[2].limitMax) arg0 = axies[2].limitMax;

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
void Stepper::onestep(int motor) {
#ifdef VERBOSE
  Serial.print(motors[motor].letter);
#endif

  digitalWrite(motors[motor].step_pin, HIGH);
  digitalWrite(motors[motor].step_pin, LOW);
}

bool Stepper::isBlockBusy(const Segment *block) {
  return block == working_block;
}

// Process pulsing in the isr step
void Stepper::isrPulsePhase() {
  if(!working_block) return;

  const uint32_t pendingSteps = steps_total - steps_taken;
  uint8_t stepsToDo = _MIN(pendingSteps,isr_step_multiplier);
  steps_taken+=stepsToDo;

#if MACHINE_STYLE == SIXI
  if(TEST(sensorManager.positionErrorFlags, POSITION_ERROR_FLAG_ESTOP)) {
    // check if the sensor position differs from the estimated position.
    float fraction = (float)steps_taken / (float)steps_total;

    for (ALL_SENSORS(i)) {
      // interpolate live = (b-a)*f + a
      working_block->a[i].expectedPosition =
          (working_block->a[i].positionEnd - working_block->a[i].positionStart) * fraction +
          working_block->a[i].positionStart;

      float diff = abs(working_block->a[i].expectedPosition - sensorManager.sensors[i].angle);
      if(diff > POSITION_EPSILON) {
        // do nothing while the margin is too big.
        // Only end this condition is stopping the ISR, either via software disable or hardware reset.
        SET_BIT_ON(sensorManager.positionErrorFlags, POSITION_ERROR_FLAG_ERROR);
        return;
      }
    }
  }
#endif

  bool stepNeeded[NUM_MUSCLES];

  // move each axis
  do {
#ifdef DEBUG_STEPPING
    delayMicroseconds(150);
#endif

#define PULSE_PREP(NN) { \
    over##NN += delta##NN; \
    stepNeeded[NN] = (over##NN >= 0); \
    if(stepNeeded[NN]) { \
      global_steps_##NN += global_step_dir_##NN; \
      over##NN -= advance_divisor; \
    } \
  }
#define PULSE_START(NN)      if(stepNeeded[NN]) digitalWrite(MOTOR_##NN##_STEP_PIN, START##NN);
#define PULSE_FINISH(NN)     if(stepNeeded[NN]) digitalWrite(MOTOR_##NN##_STEP_PIN, END##NN);

    ALL_MOTOR_MACRO(PULSE_PREP);

#if NUM_SERVOS > 0
    servoOver0 += servoDelta0;
#endif

    ALL_MOTOR_MACRO(PULSE_START);
    // now that the pins have had a moment to settle, do the second half of the steps.
    ALL_MOTOR_MACRO(PULSE_FINISH);

#if NUM_SERVOS > 0
    // servo 0
    if(servoOver0 > 0) {
      servoOver0 -= steps_total;
      global_servoSteps_0 += global_servoStep_dir_0;

#  ifdef ESP8266
      // analogWrite(SERVO0_PIN, global_servoSteps_0);
#  elif !defined(HAS_GRIPPER)
      servos[0].write(global_servoSteps_0);
#  endif
    }
#endif
  } while( --stepsToDo);
}



#ifdef DEBUG_STEPPING
void describeSegment(Segment *block) {
    int decel   = block->steps_total - block->decel_after;
    int nominal = block->decel_after - block->accel_until;
    Serial.print("seg: ");    Serial.print((long)block, HEX);
    Serial.print("  distance: ");  Serial.print(block->distance);
    Serial.print("  nominal_speed_sqr: ");  Serial.print(block->nominal_speed_sqr);
    Serial.print("  entry_speed_sqr: ");    Serial.print(block->entry_speed_sqr);
    Serial.print("  entry_speed_max: ");    Serial.print(block->entry_speed_max_sqr);
    Serial.print("  acceleration: ");  Serial.print(block->acceleration);
    Serial.print("  accel: ");    Serial.print(block->accel_until);
    Serial.print("  nominal: ");    Serial.print(nominal);
    Serial.print("  decel: ");    Serial.println(decel);
    Serial.print("  initial_rate: ");  Serial.print(block->initial_rate);
    Serial.print("  nominal_rate: ");  Serial.print(block->nominal_rate);
    Serial.print("  final_rate: ");  Serial.print(block->final_rate);
    Serial.print("  acceleration_steps_per_s2: ");  Serial.print(block->acceleration_steps_per_s2);
    Serial.print("  acceleration_rate: ");  Serial.print(block->acceleration_rate);
    Serial.print(F("  nominal?"));   Serial.print(TEST(block->flags,BIT_FLAG_NOMINAL) != 0 ? 'Y' : 'N');
    Serial.print(F("  recalc?"));   Serial.print(TEST(block->flags,BIT_FLAG_RECALCULATE) != 0 ? 'Y' : 'N');
    Serial.print(F("  busy?"));   Serial.print(motor.isBlockBusy(block) != 0 ? 'Y' : 'N');
    
#define DESCRIBE_DELTA(NN)  { Serial.print("  ");  Serial.print(AxisNames[NN]);  }
    ALL_MOTOR_MACRO(DESCRIBE_DELTA);
    
    Serial.println();
}
#endif


// Process blocks in the isr
hal_timer_t Stepper::isrBlockPhase() {
  hal_timer_t interval = (STEPPER_TIMER_RATE) / 1000UL;

  if(working_block) {
    // Is this segment done?
    if(steps_taken >= steps_total) {
#ifdef DEBUG_STEPPING
      Serial.print("E");
#endif
      // Move on to next segment without wasting an interrupt tick.
      planner.releaseCurrentBlock();
      working_block     = NULL;
    } else {
      if(steps_taken <= accel_until) {
        // accelerating
        acc_step_rate = STEP_MULTIPLY(time_accelerating, working_block->acceleration_rate);
        acc_step_rate = _MIN(acc_step_rate, working_block->nominal_rate);
        interval = calc_interval(acc_step_rate, &isr_step_multiplier);
        time_accelerating += interval;
#ifdef DEBUG_STEPPING
        Serial.print("A");
#endif
      } else if(steps_taken > decel_after) {
        // decelerating
        uint32_t step_rate = STEP_MULTIPLY(time_decelerating, working_block->acceleration_rate);
        if(step_rate < acc_step_rate) {
          step_rate = acc_step_rate - step_rate;
          step_rate = _MAX(step_rate, working_block->final_rate);
        } else {
          step_rate = working_block->final_rate;
        }
        interval = calc_interval(step_rate, &isr_step_multiplier);
        time_decelerating += interval;
#ifdef DEBUG_STEPPING
        Serial.print("D");
#endif
      } else {
        // cruising at nominal speed (flat top of the trapezoid)
        if(isr_nominal_rate < 0) {
          isr_nominal_rate = calc_interval(working_block->nominal_rate, &isr_step_multiplier);
        }
        interval = isr_nominal_rate;
#ifdef DEBUG_STEPPING
        Serial.print("N");
#endif
      }
    }
  }

  // segment buffer empty? do nothing
  if(working_block == NULL) {
    working_block = planner.getCurrentBlock();

    if(working_block) {
#ifdef DEBUG_STEPPING
      Serial.print("S");
      //describeSegment(working_block);
#endif

      // defererencing some data so the ISR runs faster.
      steps_total = working_block->steps_total;
      steps_taken = 0;
      acc_step_rate = working_block->initial_rate;
      accel_until = working_block->accel_until;
      decel_after = working_block->decel_after;
      time_accelerating = 0;
      time_decelerating = 0;
      isr_nominal_rate = -1;

      // set the direction pins
      if(directionBits != working_block->dir) {
        setDirections(working_block->dir);
      }

      advance_divisor = steps_total << 1;

#define PREPARE_DELTA(NN) \
      delta##NN = working_block->a[NN].absdelta << 1; \
      over##NN = -steps_total;

      ALL_MOTOR_MACRO(PREPARE_DELTA);

#if NUM_SERVOS > 0
      //global_servoSteps_0 = working_block->a[NUM_MOTORS].step_count - working_block->a[NUM_MOTORS].delta_steps;
      servoDelta0 = working_block->a[NUM_MOTORS].absdelta;
      servoOver0  = -(steps_total >> 1);

#  if defined(HAS_GRIPPER)
      gripper.sendPositionRequest(working_block->a[NUM_MOTORS].step_count, 255, 32);
#  endif
#endif
      interval = calc_interval(acc_step_rate, &isr_step_multiplier);
    }
  }

  return interval;
}

HAL_STEP_TIMER_ISR {
#ifndef DEBUG_STEPPING
  Stepper::isr();
#endif
}

void Stepper::isr() {
  static hal_timer_t nextMainISR=0;

#ifdef DEBUG_STEPPING
  uint32_t time0 = micros(),time1,time2,time3;
#endif

  //digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));

  #ifndef __AVR__
    // Disable interrupts, to avoid ISR preemption while we reprogram the period
    // (AVR enters the ISR with global interrupts disabled, so no need to do it here)
    DISABLE_ISRS();
  #endif
  
  // set the timer interrupt value as big as possible so there's little chance it triggers while i'm still in the ISR.
  HAL_timer_set_compare(STEP_TIMER_NUM, hal_timer_t(HAL_TIMER_TYPE_MAX));

  uint8_t max_loops = 10;
  hal_timer_t next_isr_ticks = 0;
  hal_timer_t min_ticks;
  do {
    // Turn the interrupts back on (reduces UART delay, apparently)
    ENABLE_ISRS();

#ifndef DEBUG_STEPPING
#  ifdef HAS_TMC2130
    if(homing == true) {
      tmc2130_homing_sequence();
      nextMainISR = HOMING_OCR1A;
    } else {
#  endif
      if(!nextMainISR) isrPulsePhase();
      if(!nextMainISR) nextMainISR = isrBlockPhase();
#  ifdef HAS_TMC2130
    }
#  endif
#else
    time1 = micros();
    if(!nextMainISR) isrPulsePhase();
    time2 = micros();
    if(!nextMainISR) nextMainISR = isrBlockPhase();
    time3 = micros();
#endif  // DEBUG_STEPPING

    hal_timer_t interval = _MIN(hal_timer_t(HAL_TIMER_TYPE_MAX),nextMainISR);

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

    if(!--max_loops) next_isr_ticks = min_ticks;
    // ORC1A has been advancing while the interrupt was running.
    // if OCR1A is too close to the timer, do the step again immediately
  } while (next_isr_ticks < min_ticks);

  // set the next isr to fire at the right time.
  HAL_timer_set_compare(STEP_TIMER_NUM, hal_timer_t(next_isr_ticks));

  // turn the interrupts back on
  ENABLE_ISRS();
  
#ifdef DEBUG_STEP_TIMING
  time1-=time0;
  time2-=time0;
  time3-=time0;
  uint32_t time4=micros()-time0;
  Serial.print(" T ");
  Serial.print(time1);
  Serial.print(" ");
  Serial.print(time2);
  Serial.print(" ");
  Serial.print(time3);
  Serial.print(" ");
  Serial.println(time4);
#endif
}

void Stepper::setDirections(uint16_t bits) {
  directionBits = bits;
#define SET_STEP_DIR(NN) \
  if(!!(directionBits&(1UL<<NN))) { \
    digitalWrite(MOTOR_##NN##_DIR_PIN, STEPPER_DIR_HIGH); \
    global_step_dir_##NN = 1; \
  } else { \
    digitalWrite(MOTOR_##NN##_DIR_PIN, STEPPER_DIR_LOW); \
    global_step_dir_##NN = -1; \
  }

  ALL_MOTOR_MACRO(SET_STEP_DIR);

#if NUM_SERVOS > 0
  global_servoStep_dir_0 = (!!(working_block->dir&(1<<NUM_MOTORS))) ? -1 : 1;
#endif
}
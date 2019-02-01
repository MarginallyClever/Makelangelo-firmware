//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#include "configure.h"
#include "motor.h"
#include "MServo.h"
#include "LCD.h"


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------

Motor motors[NUM_MOTORS+NUM_SERVOS];
Servo servos[NUM_SERVOS];

Segment line_segments[MAX_SEGMENTS];
Segment *working_seg = NULL;
volatile int current_segment=0;
volatile int last_segment=0;
int step_multiplier, nominal_step_multiplier;
unsigned short nominal_OCR1A;

// used by timer1 to optimize interrupt inner loop
int steps_total;
int steps_taken;
int accel_until,decel_after;
long current_feed_rate;
long current_acceleration;
long old_feed_rate=0;
long start_feed_rate,end_feed_rate;
long time_accelerating,time_decelerating;
float max_xy_jerk = MAX_JERK;

int delta0;
int over0;
long global_steps_0;
int global_step_dir_0;
#if NUM_MOTORS>1
int delta1;
int over1;
long global_steps_1;
int global_step_dir_1;
#endif
#if NUM_MOTORS>2
int delta2;
int over2;
long global_steps_2;
int global_step_dir_2;
#endif
#if NUM_MOTORS>3
int delta3;
int over3;
long global_steps_3;
int global_step_dir_3;
#endif
#if NUM_MOTORS>4
int delta4;
int over4;
long global_steps_4;
int global_step_dir_4;
#endif
#if NUM_MOTORS>5
int delta5;
int over5;
long global_steps_5;
int global_step_dir_5;
#endif

float previous_nominal_speed=0;
float previous_safe_speed=0;
float previous_speed[NUM_MOTORS+NUM_SERVOS];

const char *MotorNames="LRUVWT";
const char *AxisNames="XYZUVWT";
float maxFeedRate[NUM_MOTORS];


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------


// for reasons I don't understand... if i put this method in the .h file i get compile errors.
// so I put it here, which forces the externs.
FORCE_INLINE Segment *segment_get_working() {
  if(current_segment == last_segment ) return NULL;
  working_seg = &line_segments[current_segment];
  working_seg->busy=true;
  return working_seg;
}


FORCE_INLINE int get_next_segment(int i) {
  return SEGMOD( i + 1 );
}


FORCE_INLINE int get_prev_segment(int i) {
  return SEGMOD( i - 1 );
}

/**
 * Calculate the maximum allowable speed at this point, in order
 * to reach 'target_velocity' using 'acceleration' within a given
 * 'distance'.
 * @param acc acceleration
 * @param target_velocity 
 * @param distance 
 */
float max_speed_allowed(const float &acc, const float &target_velocity, const float &distance) {
  return sqrt( sq(target_velocity) - 2 * acc * distance );
}


/**
 * set up the pins for each motor
 */
void motor_setup() {
  motors[0].step_pin        =MOTOR_0_STEP_PIN;
  motors[0].dir_pin         =MOTOR_0_DIR_PIN;
  motors[0].enable_pin      =MOTOR_0_ENABLE_PIN;
  motors[0].limit_switch_pin=MOTOR_0_LIMIT_SWITCH_PIN;
#if NUM_MOTORS>1
  motors[1].step_pin        =MOTOR_1_STEP_PIN;
  motors[1].dir_pin         =MOTOR_1_DIR_PIN;
  motors[1].enable_pin      =MOTOR_1_ENABLE_PIN;
  motors[1].limit_switch_pin=MOTOR_1_LIMIT_SWITCH_PIN;
#endif
#if NUM_MOTORS>2
  motors[2].step_pin        =MOTOR_2_STEP_PIN;
  motors[2].dir_pin         =MOTOR_2_DIR_PIN;
  motors[2].enable_pin      =MOTOR_2_ENABLE_PIN;
  motors[2].limit_switch_pin=MOTOR_2_LIMIT_SWITCH_PIN;
#endif
#if NUM_MOTORS>3
  motors[3].step_pin        =MOTOR_3_STEP_PIN;
  motors[3].dir_pin         =MOTOR_3_DIR_PIN;
  motors[3].enable_pin      =MOTOR_3_ENABLE_PIN;
  motors[3].limit_switch_pin=MOTOR_3_LIMIT_SWITCH_PIN;
#endif
#if NUM_MOTORS>4
  motors[4].step_pin        =MOTOR_4_STEP_PIN;
  motors[4].dir_pin         =MOTOR_4_DIR_PIN;
  motors[4].enable_pin      =MOTOR_4_ENABLE_PIN;
  motors[4].limit_switch_pin=MOTOR_4_LIMIT_SWITCH_PIN;
#endif
#if NUM_MOTORS>5
  motors[5].step_pin        =MOTOR_5_STEP_PIN;
  motors[5].dir_pin         =MOTOR_5_DIR_PIN;
  motors[5].enable_pin      =MOTOR_5_ENABLE_PIN;
  motors[5].limit_switch_pin=MOTOR_5_LIMIT_SWITCH_PIN;
#endif

  int i;
  for(i=0;i<NUM_MOTORS;++i) {
    // set the motor pin & scale
    pinMode(motors[i].step_pin,OUTPUT);
    pinMode(motors[i].dir_pin,OUTPUT);
    pinMode(motors[i].enable_pin,OUTPUT);

    // set the switch pin
    pinMode(motors[i].limit_switch_pin,INPUT);
    digitalWrite(motors[i].limit_switch_pin,HIGH);
    maxFeedRate[i] = MAX_FEEDRATE;
  }

  long steps[NUM_MOTORS+NUM_SERVOS];
  memset(steps,0,(NUM_MOTORS+NUM_SERVOS)*sizeof(long));
  
  
  motor_set_step_count(steps);

  // setup servos
#if NUM_SERVOS>0
  servos[0].attach(SERVO0_PIN);
#endif
#if NUM_SERVOS>1
  servos[1].attach(SERVO1_PIN);
#endif
#if NUM_SERVOS>2
  servos[2].attach(SERVO2_PIN);
#endif
#if NUM_SERVOS>3
  servos[3].attach(SERVO3_PIN);
#endif
#if NUM_SERVOS>4
  servos[4].attach(SERVO4_PIN);
#endif

  current_segment=0;
  last_segment=0;
  Segment &old_seg = line_segments[get_prev_segment(last_segment)];
  old_seg.a[0].step_count=0;
#if NUM_MOTORS>1
  old_seg.a[1].step_count=0;
#endif
#if NUM_MOTORS>2
  old_seg.a[2].step_count=0;
#endif
#if NUM_MOTORS>3
  old_seg.a[3].step_count=0;
#endif
#if NUM_MOTORS>4
  old_seg.a[4].step_count=0;
#endif
#if NUM_MOTORS>5
  old_seg.a[5].step_count=0;
#endif

#if NUM_SERVOS>0
  old_seg.a[NUM_MOTORS].step_count=0;
#endif

  working_seg = NULL;

  // disable global interrupts
  noInterrupts();
  // set entire TCCR1A register to 0
  TCCR1A = 0;
  // set the overflow clock to 0
  TCNT1  = 0;
  // set compare match register to desired timer count
  OCR1A = 2000;  // 1ms
  // turn on CTC mode
  TCCR1B = (1 << WGM12);
  // Set 8x prescaler
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  interrupts();  // enable global interrupts
}


// turn on power to the motors (make them immobile)
void motor_engage() {
  int i;
  for(i=0;i<NUM_MOTORS;++i) {
    digitalWrite(motors[i].enable_pin,LOW);
  }
/*
#if MACHINE_STYLE == ARM6
  // DM320T drivers want high for enabled
  digitalWrite(motors[4].enable_pin,HIGH);
  digitalWrite(motors[5].enable_pin,HIGH);
#endif*/
}


// turn off power to the motors (make them move freely)
void motor_disengage() {
  int i;
  for(i=0;i<NUM_MOTORS;++i) {
    digitalWrite(motors[i].enable_pin,HIGH);
  }/*
#if MACHINE_STYLE == ARM6
  // DM320T drivers want low for disabled
  digitalWrite(motors[4].enable_pin,LOW);
  digitalWrite(motors[5].enable_pin,LOW);
#endif*/
}


// Change pen state.
void setPenAngle(int arg0) {
#if NUM_AXIES>=3
  if(arg0 < axies[2].limitMin) arg0=axies[2].limitMin;
  if(arg0 > axies[2].limitMax) arg0=axies[2].limitMax;
  
  axies[2].pos = arg0;
  
#if NUM_SERVOS>0
  servos[0].write(arg0);
#endif

#endif
}




void recalculate_reverse2(Segment *const current,const Segment *next) {
  if(current==NULL || next==NULL) return;

  float entry_speed_max = current->entry_speed_max;
  if (current->entry_speed != entry_speed_max) {
    // If nominal length true, max junction speed is guaranteed to be reached. Only compute
    // for max allowable speed if block is decelerating and nominal length is false.
    if (current->nominal_length_flag || entry_speed_max <= next->entry_speed ) {
      current->entry_speed = entry_speed_max;
    } else {
      current->entry_speed = 
        min( entry_speed_max, max_speed_allowed(-current->acceleration,next->entry_speed,current->distance) );
    }
    current->recalculate_flag = true;
  }
}

const int movesPlanned() {
  return SEGMOD( last_segment + current_segment + MAX_SEGMENTS );
}


void recalculate_reverse() {
CRITICAL_SECTION_START
  int s = last_segment;
CRITICAL_SECTION_END

  int count = movesPlanned();
  if(count>3) {
    Segment *blocks[3] = {NULL,NULL,NULL};
    while(s != current_segment) {
      s=get_prev_segment(s);
      blocks[2]=blocks[1];
      blocks[1]=blocks[0];
      blocks[0]=&line_segments[s];
      recalculate_reverse2(blocks[1],blocks[2]);
    }
  }
}


void recalculate_forward2(const Segment *prev,Segment *const current) {
  if(prev==NULL) return;

  // If the previous block is an acceleration block, but it is not long enough to complete the
  // full speed change within the block, we need to adjust the entry speed accordingly. Entry
  // speeds have already been reset, maximized, and reverse planned by reverse planner.
  // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
  if (!prev->nominal_length_flag) {
    if (prev->entry_speed < current->entry_speed) {
      double entry_speed = min( current->entry_speed,
                                    max_speed_allowed(-prev->acceleration,prev->entry_speed,prev->distance) );

      // Check for junction speed change
      if (current->entry_speed != entry_speed) {
        current->entry_speed = entry_speed;
        current->recalculate_flag = true;
      }
    }
  }
}


void recalculate_forward() {
  int s = current_segment;

  Segment *blocks[3] = {NULL,NULL,NULL};
  while(s != last_segment) {
    s=get_next_segment(s);
    blocks[0]=blocks[1];
    blocks[1]=blocks[2];
    blocks[2]=&line_segments[s];
    recalculate_forward2(blocks[0],blocks[1]);
  }
  recalculate_forward2(blocks[1],blocks[2]);
}


float estimate_acceleration_distance(const float &initial_rate, const float &target_rate, const float &accel) {
  if (accel == 0) return 0; // accel was 0, set acceleration distance to 0
  return (sq(target_rate) - sq(initial_rate)) / (accel * 2);
}


int intersection_distance(const float &start_speed,const float &end_speed,const float &accel,const float &distance) {
  return ( 2.0*accel*distance - sq(start_speed) + sq(end_speed) ) / (4.0*accel);
}


void segment_update_trapezoid(Segment *s, const float &start_speed, const float &end_speed) {
  uint32_t intial_rate = ceil(s->nominal_rate * start_speed);
  uint32_t final_rate  = ceil(s->nominal_rate * end_speed  );
  
  if(intial_rate<MIN_FEEDRATE) intial_rate=MIN_FEEDRATE;
  if(final_rate <MIN_FEEDRATE) final_rate =MIN_FEEDRATE;

  int accel = s->acceleration_steps_per_s2;
  int accelerate_steps   =  ceil( estimate_acceleration_distance(intial_rate     , s->nominal_speed,  accel) );
  int deceleration_steps = floor( estimate_acceleration_distance(s->nominal_speed, final_rate      , -accel) );

  int plateau_steps = s->steps_total - accelerate_steps - deceleration_steps;
  if(plateau_steps<0) {
    accelerate_steps = ceil( intersection_distance( intial_rate, final_rate, accel, s->steps_total ) );
    accelerate_steps = max( accelerate_steps, 0 );
    accelerate_steps = min( accelerate_steps, s->steps_total );
    plateau_steps=0;
  }
CRITICAL_SECTION_START
  if(!s->busy) {
    s->accel_until = accelerate_steps;
    s->decel_after = accelerate_steps + plateau_steps;
    s->entry_speed = intial_rate;
    s->exit_speed = final_rate;
  }
CRITICAL_SECTION_END
}


void recalculate_trapezoids() {
  int s = current_segment;
  Segment *current;
  Segment *next = NULL;

  while(s != last_segment) {
    current = next;
    next = &line_segments[s];
    if (current) {
      // Recalculate if current block entry or exit junction speed has changed.
      if (current->recalculate_flag || next->recalculate_flag)
      {
        // NOTE: Entry and exit factors always > 0 by all previous logic operations.
        float nom = current->nominal_speed;
        segment_update_trapezoid(current,current->entry_speed/nom, next->entry_speed/nom);
        current->recalculate_flag = false; // Reset current only to ensure next trapezoid is computed
      }
    }
    s=get_next_segment(s);
  }
  // Last/newest block in buffer. Make sure the last block always ends motion.
  if(next != NULL) {
    float nom = current->nominal_speed;
    segment_update_trapezoid(next, next->entry_speed/nom, MIN_FEEDRATE/nom);
    next->recalculate_flag = false;
  }
}


void recalculate_acceleration() {
  recalculate_reverse();
  recalculate_forward();
  recalculate_trapezoids();
}

void describe_segments() {
CRITICAL_SECTION_START
  Serial.println("A = index");
  Serial.println("B = start max");
  Serial.println("C = accel");
  Serial.println("D = start speed");
  Serial.println("E = nominal speed");
  Serial.println("F = end speed");
  Serial.println("G = total");
  Serial.println("H = accel until");
  Serial.println("I = decel after");
  Serial.println("J = nominal?");
  Serial.println("\nA\tB\tC\tD\tE\tF\tG\tH\tI\tJ");
  Serial.println("---------------------------------------------");

  int s = current_segment;
  while(s != last_segment) {
    Segment *next = &line_segments[s];
    s=get_next_segment(s);
                             Serial.print(s);
    Serial.print(F("\t"));   Serial.print(next->entry_speed_max);
    Serial.print(F("\t"));   Serial.print(acceleration);
    Serial.print(F("\t"));  Serial.print(next->entry_speed);
    Serial.print(F("\t"));   Serial.print(next->nominal_speed);
    Serial.print(F("\t"));  Serial.print(next->exit_speed);
    Serial.print(F("\t"));  Serial.print(next->steps_total);
    Serial.print(F("\t"));  Serial.print(next->accel_until);
    Serial.print(F("\t"));  Serial.print(next->decel_after);
    Serial.print(F("\t"));   Serial.println(next->nominal_length_flag!=0?'Y':'N');
  }
CRITICAL_SECTION_END
}


void motor_set_step_count(long *a) {
  wait_for_empty_segment_buffer();

  for(int i=0;i<NUM_MOTORS+NUM_SERVOS;++i) {
    previous_speed[i]=0;
  }

  Segment &old_seg = line_segments[get_prev_segment(last_segment)];
  old_seg.a[0].step_count=a[0];
#if NUM_MOTORS>1
  old_seg.a[1].step_count=a[1];
#endif
#if NUM_MOTORS>2
  old_seg.a[2].step_count=a[2];
#endif
#if NUM_MOTORS>3
  old_seg.a[3].step_count=a[3];
#endif
#if NUM_MOTORS>4
  old_seg.a[4].step_count=a[4];
#endif
#if NUM_MOTORS>5
  old_seg.a[5].step_count=a[5];
#endif
#if NUM_SERVOS>0
  old_seg.a[NUM_MOTORS].step_count=a[NUM_MOTORS];
#endif

  global_steps_0=0;
#if NUM_MOTORS>1
  global_steps_1=0;
#endif
#if NUM_MOTORS>2
  global_steps_2=0;
#endif
#if NUM_MOTORS>3
  global_steps_3=0;
#endif
#if NUM_MOTORS>4
  global_steps_4=0;
#endif
#if NUM_MOTORS>5
  global_steps_5=0;
#endif
}


/**
 * Step one motor one time in the currently set direction.
 * @input newx the destination x position
 * @input newy the destination y position
 **/
void motor_onestep(int motor) {
#ifdef VERBOSE
  Serial.print(motorNames[motor]);
#endif

  digitalWrite(motors[motor].step_pin,HIGH);
  digitalWrite(motors[motor].step_pin,LOW);
}


/**
 * Set the clock 2 timer frequency.
 * @input desired_freq_hz the desired frequency
 * Different clock sources can be selected for each timer independently.
 * To calculate the timer frequency (for example 2Hz using timer1) you will need:
 */
FORCE_INLINE unsigned short calc_timer(unsigned short desired_freq_hz) {
  if( desired_freq_hz > MAX_FEEDRATE ) desired_freq_hz = MAX_FEEDRATE;
  if( desired_freq_hz < MIN_FEEDRATE ) desired_freq_hz = MIN_FEEDRATE;
  old_feed_rate = desired_freq_hz;

  if( desired_freq_hz > 20000 ) {
    step_multiplier = 4;
    desired_freq_hz >>=2;
  } else if( desired_freq_hz > 10000 ) {
    step_multiplier = 2;
    desired_freq_hz >>=1;
  } else {
    step_multiplier = 1;
  }

  long counter_value = ( CLOCK_FREQ >>3 ) / desired_freq_hz;
  if( counter_value >= MAX_COUNTER ) {
    counter_value = MAX_COUNTER - 1;
  } else if( counter_value < 100 ) {
    counter_value = 100;
  }

  return counter_value;
}


/**
 * Process all line segments in the ring buffer.  Uses bresenham's line algorithm to move all motors.
 */
ISR(TIMER1_COMPA_vect) {
  // segment buffer empty? do nothing
  if( working_seg == NULL ) {
    working_seg = segment_get_working();

    if( working_seg != NULL ) {
      // New segment!
      // set the direction pins
      digitalWrite( MOTOR_0_DIR_PIN, working_seg->a[0].dir );
      global_step_dir_0 = (working_seg->a[0].dir==HIGH)?1:-1;

      #if NUM_MOTORS>1
      digitalWrite( MOTOR_1_DIR_PIN, working_seg->a[1].dir );
      global_step_dir_1 = (working_seg->a[1].dir==HIGH)?1:-1;
      #endif
      #if NUM_MOTORS>2
      digitalWrite( MOTOR_2_DIR_PIN, working_seg->a[2].dir );
      global_step_dir_2 = (working_seg->a[2].dir==HIGH)?1:-1;
      #endif
      #if NUM_MOTORS>3
      digitalWrite( MOTOR_3_DIR_PIN, working_seg->a[3].dir );
      global_step_dir_3 = (working_seg->a[3].dir==HIGH)?1:-1;
      #endif
      #if NUM_MOTORS>4
      digitalWrite( MOTOR_4_DIR_PIN, working_seg->a[4].dir );
      global_step_dir_4 = (working_seg->a[4].dir==HIGH)?1:-1;
      #endif
      #if NUM_MOTORS>5
      digitalWrite( MOTOR_5_DIR_PIN, working_seg->a[5].dir );
      global_step_dir_5 = (working_seg->a[5].dir==HIGH)?1:-1;
      #endif

      #if NUM_SERVOS>0
      servos[0].write(working_seg->a[NUM_MOTORS].step_count);
      #endif

      // set frequency to segment feed rate
      nominal_OCR1A = calc_timer(working_seg->nominal_speed);
      nominal_step_multiplier = step_multiplier;

      start_feed_rate = working_seg->entry_speed;
      end_feed_rate = working_seg->exit_speed;
      current_feed_rate = start_feed_rate;
      current_acceleration = acceleration;
      time_decelerating = 0;
      time_accelerating = calc_timer(start_feed_rate);
      OCR1A = time_accelerating;

      // defererencing some data so the loop runs faster.
      steps_total=working_seg->steps_total;
      steps_taken=0;
      delta0 = working_seg->a[0].absdelta;      over0 = -(steps_total>>1);
      #if NUM_MOTORS>1
      delta1 = working_seg->a[1].absdelta;      over1 = -(steps_total>>1);
      #endif
      #if NUM_MOTORS>2
      delta2 = working_seg->a[2].absdelta;      over2 = -(steps_total>>1);
      #endif
      #if NUM_MOTORS>3
      delta3 = working_seg->a[3].absdelta;      over3 = -(steps_total>>1);
      #endif
      #if NUM_MOTORS>4
      delta4 = working_seg->a[4].absdelta;      over4 = -(steps_total>>1);
      #endif
      #if NUM_MOTORS>5
      delta5 = working_seg->a[5].absdelta;      over5 = -(steps_total>>1);
      #endif
      accel_until=working_seg->accel_until;
      decel_after=working_seg->decel_after;
      return;
    } else {
      OCR1A = 2000; // wait 1ms
      return;
    }
  }

  if( working_seg != NULL ) {
    // move each axis
    for(uint8_t i=0;i<step_multiplier;++i) {
      over0 += delta0;
      if(over0 > 0) digitalWrite(MOTOR_0_STEP_PIN,LOW);
#if NUM_MOTORS>1
      over1 += delta1;
      if(over1 > 0) digitalWrite(MOTOR_1_STEP_PIN,LOW);
#endif
#if NUM_MOTORS>2
      over2 += delta2;
      if(over2 > 0) digitalWrite(MOTOR_2_STEP_PIN,LOW);
#endif
#if NUM_MOTORS>3
      over3 += delta3;
      if(over3 > 0) digitalWrite(MOTOR_3_STEP_PIN,LOW);
#endif
#if NUM_MOTORS>4
      over4 += delta4;
      if(over4 > 0) digitalWrite(MOTOR_4_STEP_PIN,LOW);
#endif
#if NUM_MOTORS>5
      over5 += delta5;
      if(over5 > 0) digitalWrite(MOTOR_5_STEP_PIN,LOW);
#endif
      // now that the pins have had a moment to settle, do the second half of the steps.
      // M0
      if(over0 > 0) {
        over0 -= steps_total;
        global_steps_0+=global_step_dir_0;
        digitalWrite(MOTOR_0_STEP_PIN,HIGH);
      }
#if NUM_MOTORS>1
      // M1
      if(over1 > 0) {
        over1 -= steps_total;
        global_steps_1+=global_step_dir_1;
        digitalWrite(MOTOR_1_STEP_PIN,HIGH);
      }
#endif
#if NUM_MOTORS>2
      // M2
      if(over2 > 0) {
        over2 -= steps_total;
        global_steps_2+=global_step_dir_2;
        digitalWrite(MOTOR_2_STEP_PIN,HIGH);
      }
#endif
#if NUM_MOTORS>3
      // M3
      if(over3 > 0) {
        over3 -= steps_total;
        global_steps_3+=global_step_dir_3;
        digitalWrite(MOTOR_3_STEP_PIN,HIGH);
      }
#endif
#if NUM_MOTORS>4
      // M4
      if(over4 > 0) {
        over4 -= steps_total;
        global_steps_4+=global_step_dir_4;
        digitalWrite(MOTOR_4_STEP_PIN,HIGH);
      }
#endif
#if NUM_MOTORS>5
      // M5
      if(over5 > 0) {
        over5 -= steps_total;
        global_steps_5+=global_step_dir_5;
        digitalWrite(MOTOR_5_STEP_PIN,HIGH);
      }
#endif
      
      // make a step
      steps_taken++;
      if(steps_taken>=steps_total) break;
    }

    // accel
    unsigned short t;
    if( steps_taken <= accel_until ) {
      current_feed_rate = start_feed_rate + (current_acceleration * time_accelerating / 1000000);
      if(current_feed_rate > working_seg->nominal_speed) {
        current_feed_rate = working_seg->nominal_speed;
      }
      t = calc_timer(current_feed_rate);
      OCR1A = t;
      time_accelerating+=t;
    } else if( steps_taken > decel_after ) {
      long end_feed_rate = current_feed_rate - (current_acceleration * time_decelerating / 1000000);
      if( end_feed_rate < working_seg->exit_speed ) {
        end_feed_rate = working_seg->exit_speed;
      }
      t = calc_timer(end_feed_rate);
      OCR1A = t;
      time_decelerating+=t;
    } else {
      OCR1A = nominal_OCR1A;
      step_multiplier = nominal_step_multiplier;
    }

    OCR1A = (OCR1A < (TCNT1 + 16)) ? (TCNT1 + 16) : OCR1A;
    
    // Is this segment done?
    if( steps_taken >= steps_total ) {
      // Move on to next segment without wasting an interrupt tick.
      working_seg = NULL;
      current_segment = get_next_segment(current_segment);
    }
  }
}


/**
 * @return 1 if buffer is full, 0 if it is not.
 */
char segment_buffer_full() {
  int next_segment = get_next_segment(last_segment);
  return (next_segment == current_segment);
}


/**
 * Translate the XYZ through the IK to get the number of motor steps and move the motors.
 * Uses bresenham's line algorithm to move both motors
 * @input pos NUM_AXIES floats describing destination coordinates
 * @input new_feed_rate speed to travel along arc
 */
void motor_line(const float * const target_position,float &fr_mm_s) {
  long steps[NUM_MOTORS + NUM_SERVOS];
  IK(target_position, steps);

  int i;
  for(i=0;i<NUM_AXIES;++i) {
    axies[i].pos = target_position[i];
  }
  
  feed_rate = fr_mm_s;

  // get the next available spot in the segment buffer
  int next_segment = get_next_segment(last_segment);
  while( next_segment == current_segment ) {
    // the buffer is full, we are way ahead of the motion system
    delay(1);
  }

  int prev_segment = get_prev_segment(last_segment);
  Segment &new_seg = line_segments[last_segment];
  Segment &old_seg = line_segments[prev_segment];

/*
  int k;
  for(k=0;k<NUM_MOTORS+NUM_SERVOS;++k) {
    Serial.print(n[k]);
    Serial.print(",\t");
  }
  Serial.print('\n');//*/
  
  // use LCD to adjust speed while drawing
#ifdef HAS_LCD
  fr_mm_s *= (float)speed_adjust * 0.01f;
#endif

  new_seg.a[0].step_count = steps[0];
  new_seg.a[0].delta = steps[0] - old_seg.a[0].step_count;
#if NUM_MOTORS>1
  new_seg.a[1].step_count = steps[1];
  new_seg.a[1].delta = steps[1] - old_seg.a[1].step_count;
#endif
#if NUM_MOTORS>2
  new_seg.a[2].step_count = steps[2];
  new_seg.a[2].delta = steps[2] - old_seg.a[2].step_count;
#endif
#if NUM_MOTORS>3
  new_seg.a[3].step_count = steps[3];
  new_seg.a[3].delta = steps[3] - old_seg.a[3].step_count;
#endif
#if NUM_MOTORS>4
  new_seg.a[4].step_count = steps[4];
  new_seg.a[4].delta = steps[4] - old_seg.a[4].step_count;
#endif
#if NUM_MOTORS>5
  new_seg.a[5].step_count = steps[5];
  new_seg.a[5].delta = steps[5] - old_seg.a[5].step_count;
#endif

#if NUM_SERVOS>0
  new_seg.a[NUM_MOTORS].step_count = steps[NUM_MOTORS];
  new_seg.a[NUM_MOTORS].delta = steps[NUM_MOTORS] - old_seg.a[NUM_MOTORS].step_count;
#endif

  new_seg.busy=false;

  // the axis that has the most steps will control the overall acceleration
  new_seg.steps_total = 0;
  float distance_mm=0;
  for(i=0;i<NUM_MOTORS+NUM_SERVOS;++i) {
    new_seg.a[i].dir = ( new_seg.a[i].delta < 0 ? HIGH : LOW );
    new_seg.a[i].absdelta = abs(new_seg.a[i].delta);
    distance_mm += sq(new_seg.a[i].delta*THREAD_PER_STEP);
    if( new_seg.steps_total < new_seg.a[i].absdelta ) {
      new_seg.steps_total = new_seg.a[i].absdelta;
    }
  }
  
  // No steps?  No work!  Stop now.
  if( new_seg.steps_total == 0 ) return;
  
  new_seg.distance = sqrt( distance_mm );
  float inverse_distance_mm = 1.0 / new_seg.distance;
  float inverse_mm_s = fr_mm_s * inverse_distance_mm;
  new_seg.nominal_speed = new_seg.distance * inverse_mm_s;
  new_seg.nominal_rate = ceil(new_seg.steps_total * inverse_mm_s);
  
  int movesQueued = movesPlanned();
  
  Serial.print("distance=");  Serial.println(new_seg.distance);
  Serial.print("inverse_distance_mm=");  Serial.println(inverse_distance_mm);
  Serial.print("inverse_mm_s=");  Serial.println(inverse_mm_s);
  Serial.print("nominal_speed=");  Serial.println(new_seg.nominal_speed);
  Serial.print("nominal_rate=");  Serial.println(new_seg.nominal_rate);
  
  float current_speed[NUM_MOTORS+NUM_SERVOS], speed_factor = 1.0;
  
  for(i=0;i<NUM_MOTORS+NUM_SERVOS;++i) {
    current_speed[i] = new_seg.a[i].delta * inverse_mm_s;
    const float cs = fabs(current_speed[i]);
    //if(cs>max_feedrate_mm_s[i]) speed_factor = min (speed_factor, max_feedrate_mm_s[i]/cs);
    if(cs>MAX_FEEDRATE) speed_factor = min (speed_factor, MAX_FEEDRATE/cs);
  }
  if(speed_factor<1.0) {
    for(i=0;i<NUM_MOTORS+NUM_SERVOS;++i) {
      current_speed[i] *= speed_factor;
    }
    new_seg.nominal_speed *= speed_factor;
    new_seg.nominal_rate *= speed_factor;
  }
  
  const float steps_per_mm = new_seg.steps_total * inverse_distance_mm;
  uint32_t accel = ceil( acceleration ) * steps_per_mm;
  
  for(i=0;i<NUM_MOTORS+NUM_SERVOS;++i) {
    if(new_seg.a[i].absdelta && MAX_ACCELERATION < accel) {
      const uint32_t comp = MAX_ACCELERATION * new_seg.steps_total;
      if(accel * new_seg.a[i].absdelta > comp ) {
        accel = comp / new_seg.a[i].absdelta;
      }
    }
  }
  new_seg.acceleration_steps_per_s2 = accel;
  new_seg.acceleration = acceleration / steps_per_mm;
  
  Serial.print("acceleration_steps_per_s2=");  Serial.println(new_seg.acceleration_steps_per_s2);
  Serial.print("acceleration=");  Serial.println(new_seg.acceleration);
  
  new_seg.steps_taken = 0;

  // TODO explain this
  float safe_speed = new_seg.nominal_speed;
  char limited=0;
  for(i=0;i<NUM_MOTORS+NUM_SERVOS;++i) {
    const float jerk = fabs(current_speed[i]), maxj = MAX_JERK;
    if(jerk>maxj) {
      if(limited) {
        // TODO explain this
        const float mjerk = maxj * new_seg.nominal_speed;
        if(jerk * safe_speed > mjerk) safe_speed = mjerk / jerk;
      } else {
        ++limited;
        safe_speed = maxj;
      }
    }
  }

  
  // what is the maximum starting speed for this segment?
  float vmax_junction = MIN_FEEDRATE;

  if(movesQueued>1 && previous_safe_speed>0.0001) {
    
    // Estimate a maximum velocity allowed at a joint of two successive segments.
    // If this maximum velocity allowed is lower than the minimum of the entry / exit safe velocities,
    // then the machine is not coasting anymore and the safe entry / exit velocities shall be used.

    // The junction velocity will be shared between successive segments. Limit the junction velocity to their minimum.
    bool prev_speed_larger = previous_nominal_speed > new_seg.nominal_speed;
    float smaller_speed_factor = prev_speed_larger 
        ? (new_seg.nominal_speed / previous_nominal_speed) 
        : (previous_nominal_speed / new_seg.nominal_speed);
    // Pick the smaller of the nominal speeds. Higher speed shall not be achieved at the junction during coasting.
    vmax_junction = prev_speed_larger ? new_seg.nominal_speed : previous_nominal_speed;
    // Factor to multiply the previous / current nominal velocities to get componentwise limited velocities.
    float v_factor = 1.f;
    limited = 0;
    // Now limit the jerk in all axes.
    for(i=0;i<NUM_MOTORS+NUM_SERVOS;++i) {
      // Limit an axis. We have to differentiate: coasting, reversal of an axis, full stop.
      float v_exit = previous_speed[i];
      float v_entry = current_speed[i];
      if (prev_speed_larger) v_exit *= smaller_speed_factor;
      if (limited) {
        v_exit *= v_factor;
        v_entry *= v_factor;
      }

      // Calculate jerk depending on whether the axis is coasting in the same direction or reversing.
      const float jerk = (v_exit > v_entry)
          ? //                                  coasting             axis reversal
            ( (v_entry > 0.f || v_exit < 0.f) ? (v_exit - v_entry) : max(v_exit, -v_entry) )
          : // v_exit <= v_entry                coasting             axis reversal
            ( (v_entry < 0.f || v_exit > 0.f) ? (v_entry - v_exit) : max(-v_exit, v_entry) );

      if (jerk > MAX_JERK) {
        v_factor *= MAX_JERK / jerk;
        ++limited;
      }
    }
    if (limited) vmax_junction *= v_factor;
    // Now the transition velocity is known, which maximizes the shared exit / entry velocity while
    // respecting the jerk factors, it may be possible, that applying separate safe exit / entry velocities will achieve faster prints.
    const float vmax_junction_threshold = vmax_junction * 0.99f;
    if (previous_safe_speed > vmax_junction_threshold && safe_speed > vmax_junction_threshold) {
      // Not coasting. The machine will stop and start the movements anyway,
      // better to start the segment from start.
      //SBI(new_seg.flag, BLOCK_BIT_START_FROM_FULL_HALT);
      vmax_junction = safe_speed;
    }
  }

  float allowable_speed = max_speed_allowed(-new_seg.acceleration, MIN_FEEDRATE, new_seg.distance);

#if NUM_SERVOS>0
  // come to a stop for entering or exiting a Z move
  //if( new_seg.a[NUM_SERVOS].delta != 0 || old_seg.a[NUM_SERVOS].delta != 0 ) allowable_speed = MIN_FEEDRATE;
#endif

  Serial.print("Allowed speed=");
  Serial.println(allowable_speed);
  
  Serial.print("nominal_speed=");
  Serial.println(new_seg.nominal_speed);
  
  new_seg.entry_speed_max = vmax_junction;
  new_seg.entry_speed = min(vmax_junction, allowable_speed);

  new_seg.nominal_length_flag = ( allowable_speed >= new_seg.nominal_speed );
  new_seg.recalculate_flag = true;

  previous_nominal_speed = new_seg.nominal_speed;
  previous_safe_speed = safe_speed;
  
  // when should we accelerate and decelerate in this segment?
  segment_update_trapezoid(&new_seg,new_seg.entry_speed/new_seg.nominal_speed,MIN_FEEDRATE/new_seg.nominal_speed);

  last_segment = next_segment;

  recalculate_acceleration();
  describe_segments();
}


void wait_for_empty_segment_buffer() {
  while( current_segment != last_segment );
}

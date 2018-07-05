//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#include "MServo.h"


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
float max_speed_allowed(float acc, float target_velocity, float distance) {
  return sqrt( target_velocity*target_velocity - 2 * acc * distance );
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




void recalculate_reverse2(Segment *prev,Segment *current,Segment *next) {
  if(current==NULL) return;
  if(next==NULL) return;

  if (current->feed_rate_start != current->feed_rate_start_max) {
    // If nominal length true, max junction speed is guaranteed to be reached. Only compute
    // for max allowable speed if block is decelerating and nominal length is false.
    if ((!current->nominal_length_flag) && (current->feed_rate_start_max > next->feed_rate_start)) {
      float v = min( current->feed_rate_start_max,
                     max_speed_allowed(-acceleration,next->feed_rate_start,current->steps_total));
      current->feed_rate_start = v;
    } else {
      current->feed_rate_start = current->feed_rate_start_max;
    }
    current->recalculate_flag = true;
  }
}


void recalculate_reverse() {
CRITICAL_SECTION_START
  int s = last_segment;
CRITICAL_SECTION_END

  Segment *blocks[3] = {NULL,NULL,NULL};
  int count = SEGMOD( last_segment + current_segment + MAX_SEGMENTS );
  if(count>3) {
    while(s != current_segment) {
      s=get_prev_segment(s);
      blocks[2]=blocks[1];
      blocks[1]=blocks[0];
      blocks[0]=&line_segments[s];
      recalculate_reverse2(blocks[0],blocks[1],blocks[2]);
    }
  }
}


void recalculate_forward2(Segment *prev,Segment *current,Segment *next) {
  if(prev==NULL) return;

  // If the previous block is an acceleration block, but it is not long enough to complete the
  // full speed change within the block, we need to adjust the entry speed accordingly. Entry
  // speeds have already been reset, maximized, and reverse planned by reverse planner.
  // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
  if (!prev->nominal_length_flag) {
    if (prev->feed_rate_start < current->feed_rate_start) {
      double feed_rate_start = min( current->feed_rate_start,
                                    max_speed_allowed(-acceleration,prev->feed_rate_start,prev->steps_total) );

      // Check for junction speed change
      if (current->feed_rate_start != feed_rate_start) {
        current->feed_rate_start = feed_rate_start;
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
    recalculate_forward2(blocks[0],blocks[1],blocks[2]);
  }
  recalculate_forward2(blocks[1],blocks[2],NULL);
}


int intersection_time(float accel,float distance,float start_speed,float end_speed) {
#if 0
  return ( ( 2.0*accel*distance - start_speed*start_speed + end_speed*end_speed ) / (4.0*accel) );
#else
  float t2 = ( start_speed - end_speed + accel * distance ) / ( 2.0 * accel );
  return distance - t2;
#endif
}


void segment_update_trapezoid(Segment *s,float start_speed,float end_speed) {
  if(start_speed<MIN_FEEDRATE) start_speed=MIN_FEEDRATE;
  if(end_speed<MIN_FEEDRATE) end_speed=MIN_FEEDRATE;

  int steps_to_accel =  ceil( (s->feed_rate_max*s->feed_rate_max - start_speed*start_speed )/ (2.0*acceleration) );
  int steps_to_decel = floor( (end_speed*end_speed - s->feed_rate_max*s->feed_rate_max )/ -(2.0*acceleration) );
  //int steps_to_accel =  ceil( ( s->feed_rate_max - start_speed ) / acceleration );
  //int steps_to_decel = floor( ( end_speed - s->feed_rate_max ) / -acceleration );

  int steps_at_top_speed = s->steps_total - steps_to_accel - steps_to_decel;
  if(steps_at_top_speed<0) {
    steps_to_accel = ceil( intersection_time(acceleration,s->steps_total,start_speed,end_speed) );
    steps_to_accel = max(steps_to_accel,0);
    steps_to_accel = min(steps_to_accel,s->steps_total);
    steps_at_top_speed=0;
  }
/*
  Serial.print("M");  Serial.println(s->feed_rate_max);
  Serial.print("E");  Serial.println(end_speed);
  Serial.print("S");  Serial.println(start_speed);
  Serial.print("@");  Serial.println(acceleration);
  Serial.print("A");  Serial.println(steps_to_accel);
  Serial.print("D");  Serial.println(steps_to_decel);
*/
CRITICAL_SECTION_START
  if(!s->busy) {
    s->accel_until = steps_to_accel;
    s->decel_after = steps_to_accel+steps_at_top_speed;
    s->feed_rate_start = start_speed;
    s->feed_rate_end = end_speed;
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
        segment_update_trapezoid(current,current->feed_rate_start, next->feed_rate_start);
        current->recalculate_flag = false; // Reset current only to ensure next trapezoid is computed
      }
    }
    s=get_next_segment(s);
  }
  // Last/newest block in buffer. Make sure the last block always ends motion.
  if(next != NULL) {
    segment_update_trapezoid(next, next->feed_rate_start, MIN_FEEDRATE);
    next->recalculate_flag = false;
  }
}


void recalculate_acceleration() {
  recalculate_reverse();
  recalculate_forward();
  recalculate_trapezoids();

#if VERBOSE > 1
  //Serial.println("\nstart max,max,start,end,rate,total,up steps,cruise,down steps,nominal?");
  Serial.println("---------------");
  int s = current_segment;

  while(s != last_segment) {
    Segment *next = &line_segments[s];
    s=get_next_segment(s);
//                             Serial.print(next->feed_rate_start_max);
//    Serial.print(F("\t"));   Serial.print(next->feed_rate_max);
//    Serial.print(F("\t"));   Serial.print(acceleration);
    Serial.print(F("\tS"));  Serial.print(next->feed_rate_start);
//    Serial.print(F("\tE"));  Serial.print(next->feed_rate_end);
    Serial.print(F("\t*"));  Serial.print(next->steps_total);
    Serial.print(F("\tA"));  Serial.print(next->accel_until);
    int after = (next->steps_total - next->decel_after);
    int total = next->steps_total - after - next->accel_until;
    Serial.print(F("\tT"));  Serial.print(total);
    Serial.print(F("\tD"));  Serial.print(after);
    Serial.print(F("\t"));   Serial.println(next->nominal_length_flag==1?'*':' ');
  }
#endif
}


void motor_set_step_count(long *a) {
  wait_for_empty_segment_buffer();

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
 * Supports movement with both styles of Motor Shield
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
    desired_freq_hz = (desired_freq_hz>>2)&0x3fff;
  } else if( desired_freq_hz > 10000 ) {
    step_multiplier = 2;
    desired_freq_hz = (desired_freq_hz>>1)&0x7fff;
  } else {
    step_multiplier = 1;
  }

  long counter_value = ( CLOCK_FREQ / 8 ) / desired_freq_hz;
  if( counter_value >= MAX_COUNTER ) {
    //Serial.print("this breaks the timer and crashes the arduino");
    //Serial.flush();
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
      nominal_OCR1A = calc_timer(working_seg->feed_rate_max);
      nominal_step_multiplier = step_multiplier;

      start_feed_rate = working_seg->feed_rate_start;
      end_feed_rate = working_seg->feed_rate_end;
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
      if(current_feed_rate > working_seg->feed_rate_max) {
        current_feed_rate = working_seg->feed_rate_max;
      }
      t = calc_timer(current_feed_rate);
      OCR1A = t;
      time_accelerating+=t;
    } else if( steps_taken > decel_after ) {
      long end_feed_rate = current_feed_rate - (current_acceleration * time_decelerating / 1000000);
      if( end_feed_rate < working_seg->feed_rate_end ) {
        end_feed_rate = working_seg->feed_rate_end;
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
 * Uses bresenham's line algorithm to move both motors
 * @param n (NUM_MOTORS+NUM_SERVOS) number of steps, one for each motor/servo
 **/
void motor_line(long *n,float new_feed_rate) {
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
  new_feed_rate *= (float)speed_adjust * 0.01f;
#endif

  new_seg.a[0].step_count = n[0];
  new_seg.a[0].delta = n[0] - old_seg.a[0].step_count;
#if NUM_MOTORS>1
  new_seg.a[1].step_count = n[1];
  new_seg.a[1].delta = n[1] - old_seg.a[1].step_count;
#endif
#if NUM_MOTORS>2
  new_seg.a[2].step_count = n[2];
  new_seg.a[2].delta = n[2] - old_seg.a[2].step_count;
#endif
#if NUM_MOTORS>3
  new_seg.a[3].step_count = n[3];
  new_seg.a[3].delta = n[3] - old_seg.a[3].step_count;
#endif
#if NUM_MOTORS>4
  new_seg.a[4].step_count = n[4];
  new_seg.a[4].delta = n[4] - old_seg.a[4].step_count;
#endif
#if NUM_MOTORS>5
  new_seg.a[5].step_count = n[5];
  new_seg.a[5].delta = n[5] - old_seg.a[5].step_count;
#endif

#if NUM_SERVOS>0
  new_seg.a[NUM_MOTORS].step_count = n[NUM_MOTORS];
  new_seg.a[NUM_MOTORS].delta = n[NUM_MOTORS] - old_seg.a[NUM_MOTORS].step_count;
#endif

  new_seg.feed_rate_max = new_feed_rate;
  new_seg.busy=false;

  // the axis that has the most steps will control the overall acceleration
  new_seg.steps_total = 0;
  float len=0;
  int i;
  for(i=0;i<NUM_MOTORS+NUM_SERVOS;++i) {
    new_seg.a[i].dir = ( new_seg.a[i].delta < 0 ? HIGH : LOW );
    new_seg.a[i].absdelta = abs(new_seg.a[i].delta);
    len += square(new_seg.a[i].delta);
    if( new_seg.steps_total < new_seg.a[i].absdelta ) {
      new_seg.steps_total = new_seg.a[i].absdelta;
    }
    //Serial.print(i);
    //Serial.print('\t');    Serial.print(n[i]);
    //Serial.print('\t');    Serial.print(old_seg.a[i].step_count);
    //Serial.print('\t');    Serial.print(new_seg.a[i].step_count);
    //Serial.print('\t');    Serial.print(new_seg.a[i].dir);
    //Serial.print('\t');    Serial.print(new_seg.a[i].absdelta);
    //Serial.println();
  }

  // No steps?  No work!  Stop now.
  if( new_seg.steps_total == 0 ) return;

  //Serial.println(new_seg.steps_total);

  len = sqrt( len );
  float ilen = 1.0f / len;
  float iSecond = new_feed_rate * ilen;
  
  for(i=0;i<NUM_MOTORS;++i) {
    new_seg.a[i].delta_normalized = new_seg.a[i].delta * ilen;
  }
  new_seg.steps_taken = 0;

  // what is the maximum starting speed for this segment?
  float feed_rate_start_max = MIN_FEEDRATE;
  // is the robot changing direction sharply?
  // aka is there a previous segment with a wildly different delta_normalized?
  if(last_segment != current_segment) {
    float sum=0, d;
    d = new_seg.a[0].delta_normalized - old_seg.a[0].delta_normalized;    sum += d*d;
#if NUM_MOTORS>1
    d = new_seg.a[1].delta_normalized - old_seg.a[1].delta_normalized;    sum += d*d;
#endif
#if NUM_MOTORS>2
    d = new_seg.a[2].delta_normalized - old_seg.a[2].delta_normalized;    sum += d*d;
#endif
#if NUM_MOTORS>3
    d = new_seg.a[3].delta_normalized - old_seg.a[3].delta_normalized;    sum += d*d;
#endif
#if NUM_MOTORS>4
    d = new_seg.a[4].delta_normalized - old_seg.a[4].delta_normalized;    sum += d*d;
#endif
#if NUM_MOTORS>5
    d = new_seg.a[5].delta_normalized - old_seg.a[5].delta_normalized;    sum += d*d;
#endif

#if NUM_SERVOS>0
    d = new_seg.a[NUM_SERVOS].delta_normalized - old_seg.a[NUM_SERVOS].delta_normalized;    sum += d*d;
#endif

    
    float jerk = sqrt(sum);
    float vmax_junction_factor = 1.0;
    if(jerk> max_xy_jerk) {
      vmax_junction_factor = max_xy_jerk / jerk;
    }
    feed_rate_start_max = min( new_seg.feed_rate_max * vmax_junction_factor, old_seg.feed_rate_max );
  }

  float allowable_speed = max_speed_allowed(-acceleration, MIN_FEEDRATE, new_seg.steps_total);

#if NUM_SERVOS>0
  // come to a stop for entering or exiting a Z move
  //if( new_seg.a[NUM_SERVOS].delta != 0 || old_seg.a[NUM_SERVOS].delta != 0 ) allowable_speed = MIN_FEEDRATE;
#endif

  //Serial.print("max = ");  Serial.println(feed_rate_start_max);
//  Serial.print("allowed = ");  Serial.println(allowable_speed);
  new_seg.feed_rate_start_max = feed_rate_start_max;
  new_seg.feed_rate_start = min(feed_rate_start_max, allowable_speed);

  new_seg.nominal_length_flag = ( allowable_speed >= new_seg.feed_rate_max );
  new_seg.recalculate_flag = true;

  // when should we accelerate and decelerate in this segment?
  segment_update_trapezoid(&new_seg,new_seg.feed_rate_start,MIN_FEEDRATE);

  last_segment = next_segment;

  recalculate_acceleration();
}


void wait_for_empty_segment_buffer() {
  while( current_segment != last_segment );
}


/**
 * This file is part of makelangelo-firmware.
 *
 * makelangelo-firmware is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * makelangelo-firmware is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with makelangelo-firmware.  If not, see <http://www.gnu.org/licenses/>.
 */

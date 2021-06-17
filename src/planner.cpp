// Code that converts linear absolute moves (degree/mm) into relative blocks of steps-per-motor, then
// inserts those blocks into the block ring buffer, and optimizes acceleration between the blocks.
#include "configure.h"
#include "lcd.h"
#include "motor.h"

#define BLOCK_DELAY_FOR_1ST_MOVE 100
#define MIN_STEP_RATE            120
#define GRAVITYmag               (9800.0)  // mm/s/s

extern Planner planner;

Segment Planner::blockBuffer[MAX_SEGMENTS];
volatile int Planner::block_buffer_head, 
             Planner::block_buffer_nonbusy,
             Planner::block_buffer_planned,
             Planner::block_buffer_tail;
int Planner::first_segment_delay;

float Planner::previous_nominal_speed_sqr;
float Planner::previous_safe_speed;
float Planner::previous_speed[NUM_MUSCLES];


// returns angle of dy/dx as a value from 0...2PI
float atan3(float dy, float dx) {
  float a = atan2(dy, dx);
  if(a < 0) a += (PI * 2.0);
  return a;
}

void Planner::wait_for_empty_segment_buffer() {
  while (block_buffer_tail != block_buffer_head);
}


void Planner::zeroSpeeds() {
  wait_for_empty_segment_buffer();

  previous_nominal_speed_sqr = 0;
  previous_safe_speed = 0;
  for (ALL_MUSCLES(i)) previous_speed[i] = 0;

  block_buffer_tail = 0;
  block_buffer_head = 0;
  first_segment_delay = 0;
}

/**
   Calculate the maximum allowable speed at this point, in order
   to reach 'target_velocity' using 'acceleration' within a given
   'distance'.
   @param acc acceleration
   @param target_velocity
   @param distance
*/
float Planner::max_speed_allowed_sqr(const float &acc, const float &target_velocity_sqr, const float &distance) {
  return target_velocity_sqr - 2 * acc * distance;
}

void Planner::recalculate_reverse_kernel(Segment *const current, const Segment *next) {
  if(current == NULL) return;

  const float entry_speed_max_sqr = current->entry_speed_max_sqr;
  if(current->entry_speed_sqr != entry_speed_max_sqr || (next && TEST(next->flags,BIT_FLAG_RECALCULATE)) ) {
    // If nominal length true, max junction speed is guaranteed to be reached. Only compute
    // for max allowable speed if block is decelerating and nominal length is false.
    const float new_entry_speed_sqr = TEST(current->flags, BIT_FLAG_NOMINAL)
                                  ? entry_speed_max_sqr
                                  : min( entry_speed_max_sqr, max_speed_allowed_sqr(-current->acceleration, (next ? next->entry_speed_sqr : sq(float(MIN_FEEDRATE))), current->distance) );
    if( current->entry_speed_sqr != new_entry_speed_sqr ) {
      SET_BIT_ON(current->flags, BIT_FLAG_RECALCULATE);
      if( motor.isBlockBusy(current) ) {
        SET_BIT_OFF(current->flags, BIT_FLAG_RECALCULATE);
      } else {
        current->entry_speed_sqr = new_entry_speed_sqr;
      }
    }
  }
}

void Planner::reversePass() {
  int block_index = getPrevBlock(block_buffer_head);

  uint8_t planned_block_index = block_buffer_planned;
  if(planned_block_index == block_buffer_head) return;

  Segment *next = NULL;
  while (block_index != block_buffer_tail) {
    Segment *current = &blockBuffer[block_index];

    recalculate_reverse_kernel(current, next);
    next = current;
    block_index = getPrevBlock(block_index);
    while(planned_block_index != block_buffer_planned) {
      // If we reached the busy block or an already processed block, break the loop now
      if(block_index == planned_block_index) return;
      // Advance the pointer, following the busy block
      planned_block_index = getNextBlock(planned_block_index);
    }
  }
}

void Planner::recalculate_forward_kernel(const Segment *prev, Segment *const current,uint8_t block_index) {
  if(prev == NULL) return;

  // If the previous block is an acceleration block, but it is not long enough to complete the
  // full speed change within the block, we need to adjust the entry speed accordingly. Entry
  // speeds have already been reset, maximized, and reverse planned by reverse planner.
  // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
  if( !TEST(prev->flags,BIT_FLAG_NOMINAL) && prev->entry_speed_sqr < current->entry_speed_sqr ) {
    const float new_entry_speed_sqr = max_speed_allowed_sqr(-prev->acceleration, prev->entry_speed_sqr, prev->distance);
    // Check for junction speed change
    if(new_entry_speed_sqr < current->entry_speed_sqr) {
      SET_BIT_ON(current->flags, BIT_FLAG_RECALCULATE);
      if(motor.isBlockBusy(current)) {
        SET_BIT_OFF(current->flags, BIT_FLAG_RECALCULATE);
      } else {
        current->entry_speed_sqr = new_entry_speed_sqr;
        block_buffer_planned = block_index;
      }
    }
  }
}

void Planner::forwardPass() {
  int block_index = block_buffer_planned;

  Segment *previous = NULL;
  while (block_index != block_buffer_head) {
    Segment *current = &blockBuffer[block_index];
    if(!previous || !motor.isBlockBusy(previous)) {
      recalculate_forward_kernel(previous, current,block_index);
    }
    previous = current;
    block_index = getNextBlock(block_index);
  }
}

float Planner::estimate_acceleration_distance(const float &initial_rate, const float &target_rate, const float &accel) {
  if(accel == 0) return 0;
  return (sq(target_rate) - sq(initial_rate)) / (accel * 2);
}

int Planner::intersection_distance(const float &start_rate, const float &end_rate, const float &accel, const float &distance) {
  if(accel == 0) return 0;
  return (2.0 * accel * distance - sq(start_rate) + sq(end_rate)) / (4.0 * accel);
}

void Planner::calculate_trapezoid_for_block(Segment *s, const float &entry_factor, const float &exit_factor) {
  uint32_t intial_rate = ceil(s->nominal_rate * entry_factor);
  uint32_t final_rate  = ceil(s->nominal_rate * exit_factor);

  if(intial_rate < MIN_STEP_RATE) intial_rate = MIN_STEP_RATE;
  if(final_rate < MIN_STEP_RATE) final_rate = MIN_STEP_RATE;

  const int32_t accel = s->acceleration_steps_per_s2;
  uint32_t accelerate_steps = ceil(estimate_acceleration_distance(intial_rate, s->nominal_rate, accel));
  uint32_t decelerate_steps = floor(estimate_acceleration_distance(s->nominal_rate, final_rate, -accel));
  int32_t plateau_steps = s->steps_total - accelerate_steps - decelerate_steps;
  if(plateau_steps < 0) {
    const float accelerate_steps_float = ceil(intersection_distance(intial_rate, final_rate, accel, s->steps_total));
    accelerate_steps = min(((uint32_t)max(accelerate_steps_float, 0.0f)), s->steps_total);
    plateau_steps = 0;
  }
  s->accel_until  = accelerate_steps;
  s->decel_after  = accelerate_steps + plateau_steps;
  s->initial_rate = intial_rate;
  s->final_rate   = final_rate;
}

void Planner::recalculate_trapezoids() {
  uint8_t block_index = block_buffer_tail;
  uint8_t head_block_index = block_buffer_head;
  Segment *block = NULL;
  Segment *next = NULL;

  float current_entry_speed = 0, next_entry_speed = 0;

  while (block_index != head_block_index) {
    next             = &blockBuffer[block_index];
    next_entry_speed = sqrt(next->entry_speed_sqr);
    if(block) {
      // Recalculate if current block entry or exit junction speed has changed.
      if( TEST(block->flags,BIT_FLAG_RECALCULATE) || TEST(next->flags,BIT_FLAG_RECALCULATE) ) {
        SET_BIT_ON( block->flags, BIT_FLAG_RECALCULATE );
        if( !motor.isBlockBusy(block) ) {
          // NOTE: Entry and exit factors always > 0 by all previous logic operations.
          const float inom = 1.0 / sqrt(block->nominal_speed_sqr);
          calculate_trapezoid_for_block(block, current_entry_speed * inom, next_entry_speed * inom);
        }
      }
      // Reset current only to ensure next trapezoid is computed
      SET_BIT_OFF(block->flags,BIT_FLAG_RECALCULATE);
    }
    block_index = getNextBlock(block_index);
    current_entry_speed = next_entry_speed;
    block = next;
  }

  // Last/newest block in buffer. Make sure the last block always ends motion.
  if(next) {
    SET_BIT_ON(next->flags,BIT_FLAG_RECALCULATE);
    if( !motor.isBlockBusy(block) ) {
      const float inom = 1.0 / sqrt(next->nominal_speed_sqr);
      calculate_trapezoid_for_block(next, next_entry_speed * inom, MIN_FEEDRATE * inom);
    }
    SET_BIT_OFF(next->flags,BIT_FLAG_RECALCULATE);
  }
}

void Planner::recalculate() {
  if(getPrevBlock(block_buffer_head) != block_buffer_planned) {
    reversePass();
    forwardPass();
  }
  recalculate_trapezoids();
}

void Planner::describeAllSegments() {
  CRITICAL_SECTION_START();
  static uint8_t once = 0;
  if(once == 0) {
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
    Serial.println("R = busy?");
    Serial.println("\nA\tB\tC\tD\tE\tF\tG\tH\tI\tJ\tK\tL\tM\tN\tO\tP\tQ\tR");
  }
  Serial.println("---------------------------------------------------------------------------------------------------------------------------");

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

    Serial.print(F("\t"));   Serial.print(next->entry_speed_sqr);
    Serial.print(F("\t"));   Serial.print(next->nominal_speed_sqr);
    Serial.print(F("\t"));   Serial.print(next->entry_speed_max_sqr);

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
    Serial.print(F("\t"));   Serial.print(motor.isBlockBusy(next) != 0 ? 'Y' : 'N');
    Serial.println();
    s = getNextBlock(s);
  }
  CRITICAL_SECTION_END();
}

void Planner::addSteps(Segment *newBlock,const float *const target_position, float fr_units_s, float longest_distance) {
  int prev_segment = getPrevBlock(block_buffer_head);
  Segment &oldBlock = blockBuffer[prev_segment];

  // convert from the cartesian position to the motor steps
  long steps[NUM_MUSCLES];
  IK(target_position, steps);

#if MACHINE_STYLE == POLARGRAPH
  float oldP [] = { axies[0].pos, axies[1].pos };
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
    if(newBlock->steps_total < newBlock->a[i].absdelta) newBlock->steps_total = newBlock->a[i].absdelta;
  }

  for (ALL_AXIES(i)) axies[i].pos = target_position[i];

  // No steps?  No work!  Stop now.
  if(newBlock->steps_total < MIN_STEPS_PER_SEGMENT) return;

  newBlock->distance = longest_distance;

  // record the new target position & feed rate for the next movement.
  float inverse_distance_units = 1.0 / longest_distance;
  float inverse_secs = fr_units_s * inverse_distance_units;
  int movesQueued = movesPlannedNotBusy();
  uint32_t segment_time_us = lroundf(1000000.0f / inverse_secs);

#ifdef BUFFER_EMPTY_SLOWDOWN
    #ifndef SLOWDOWN_DIVISOR
      #define SLOWDOWN_DIVISOR 2
    #endif
  if(movesQueued >= 2 && movesQueued <= (MAX_SEGMENTS / SLOWDOWN_DIVISOR) - 1) {
    const int32_t time_diff = min_segment_time_us - segment_time_us;
    if(time_diff > 0) {
      // Serial.print("was ");  Serial.print(inverse_secs);
      const uint32_t nst = segment_time_us + lroundf(2 * time_diff  / movesQueued);
      inverse_secs       = 1000000.0f / nst;
      // Serial.print(" now ");  Serial.println(inverse_secs);
    }
  }
#endif

  newBlock->nominal_speed_sqr = sq(longest_distance * inverse_secs);
  newBlock->nominal_rate  = ceil(newBlock->steps_total * inverse_secs);

  // Calculate the the speed limit for each axis.
  // All speeds are connected so if one motor slows, they all have to slow the same amount.
  float current_speed[NUM_MUSCLES], speed_factor = 1.0;
  for (ALL_MUSCLES(i)) {
    current_speed[i] = newBlock->a[i].delta_units * inverse_secs;
    const float cs = fabs(current_speed[i]), max_fr = max_step_rate_s[i];
    if(cs > max_fr) speed_factor = min(speed_factor, max_fr / cs);
  }
  
  // apply the speed limit
  if(speed_factor < 1.0) {
    for (ALL_MUSCLES(i)) {
      current_speed[i] *= speed_factor;
    }
    newBlock->nominal_speed_sqr *= sq(speed_factor);
    newBlock->nominal_rate *= speed_factor;
  }

#if MACHINE_STYLE == POLARGRAPH && defined(DYNAMIC_ACCELERATION)
  float max_acceleration = limitPolargraphAcceleration(target_position,oldP,desiredAcceleration);
#else
  float max_acceleration = desiredAcceleration;
#endif

  const float steps_per_unit = newBlock->steps_total * inverse_distance_units;
  uint32_t accel = ceil(max_acceleration * steps_per_unit);

  uint32_t highest_rate = 1;
  float max_acceleration_steps_per_s2[NUM_MUSCLES];

  for (ALL_MUSCLES(i)) {
    max_acceleration_steps_per_s2[i] = max_acceleration * motor_spu[i];
    highest_rate = max( highest_rate, max_acceleration_steps_per_s2[i] );
  }
  uint32_t acceleration_long_cutoff = 4294967295UL / highest_rate; // 0xFFFFFFFFUL

  if(newBlock->steps_total <= acceleration_long_cutoff) {
    for (ALL_MUSCLES(i)) {
      if(newBlock->a[i].absdelta && max_acceleration_steps_per_s2[i] < accel) {
        const uint32_t max_possible = max_acceleration_steps_per_s2[i] * newBlock->steps_total / newBlock->a[i].absdelta;
        accel = min( accel, max_possible );
      }
    }
  } else {
    for (ALL_MUSCLES(i)) {
      if(newBlock->a[i].absdelta && max_acceleration_steps_per_s2[i] < accel) {
        const float max_possible = float(max_acceleration_steps_per_s2[i]) * float(newBlock->steps_total) / float(newBlock->a[i].absdelta);
        accel = min( accel, (uint32_t)max_possible );
      }
    }
  }

  newBlock->acceleration_steps_per_s2 = accel;
  newBlock->acceleration              = accel / steps_per_unit;
  newBlock->acceleration_rate         = (uint32_t)(accel * (sq(4096.0f) / (STEPPER_TIMER_RATE)));
  newBlock->steps_taken               = 0;

  // BEGIN JERK LIMITING

  CACHED_SQRT(nominal_speed,newBlock->nominal_speed_sqr);

  float safe_speed = nominal_speed;
  char limited     = 0;
  for (ALL_MUSCLES(i)) {
    const float jerk = fabs(current_speed[i]),
                maxj = max_jerk[i];
    if(jerk > maxj) {  // new current speed too fast?
      if(limited) {
        const float mjerk = maxj * nominal_speed;          // ns*mj
        if(jerk * safe_speed > mjerk) safe_speed = mjerk / jerk;  // ns*mj/cs
      } else {
        safe_speed *= maxj / jerk;  // Initial limit: ns*mj/cs
        ++limited;
      }
    }
  }

  // what is the maximum starting speed for this segment?
  float vmax_junction;
  if(movesQueued > 0 && previous_nominal_speed_sqr > 1e-6) {
    // Estimate a maximum velocity allowed at a joint of two successive segments.
    // If this maximum velocity allowed is lower than the minimum of the entry / exit safe velocities,
    // then the machine is not coasting anymore and the safe entry / exit velocities shall be used.

    // Factor to multiply the previous / current nominal velocities to get componentwise limited velocities.
    float v_factor = 1.0f;
    limited = 0;

    CACHED_SQRT(previous_nominal_speed, previous_nominal_speed_sqr);

    // The junction velocity will be shared between successive segments. Limit the junction velocity to their minimum.
    // Pick the smaller of the nominal speeds. Higher speed shall not be achieved at the junction during coasting.
    vmax_junction = min(newBlock->nominal_speed_sqr, previous_nominal_speed);
    float smaller_speed_factor = vmax_junction / previous_nominal_speed;
    // Now limit the jerk in all axes.
    for (ALL_MUSCLES(i)) {
      // Limit an axis. We have to differentiate: coasting, reversal of an axis, full stop.
      float v_exit  = previous_speed[i] * smaller_speed_factor;
      float v_entry = current_speed[i];
      if(limited) {
        v_exit *= v_factor;
        v_entry *= v_factor;
      }

      // Calculate jerk depending on whether the axis is coasting in the same direction or reversing.
      const float jerk = (v_exit > v_entry)
          ?  //                            coasting             axis reversal
          ((v_entry > 0 || v_exit < 0) ? (v_exit - v_entry) : max(v_exit, -v_entry))
          :  // v_exit <= v_entry          coasting             axis reversal
          ((v_entry < 0 || v_exit > 0) ? (v_entry - v_exit) : max(-v_exit, v_entry));

      if(jerk > max_jerk[i]) {
        v_factor *= max_jerk[i] / jerk;
        ++limited;
      }
    }

    if(limited) vmax_junction *= v_factor;
    // Now the transition velocity is known, which maximizes the shared exit / entry velocity while
    // respecting the jerk factors, it may be possible, that applying separate safe exit / entry velocities will achieve
    // faster prints.
    const float vmax_junction_threshold = vmax_junction * 0.99f;
    if(previous_safe_speed > vmax_junction_threshold && safe_speed > vmax_junction_threshold) {
      // Not coasting. The machine will stop and start the movements anyway,
      // better to start the segment from start.
      vmax_junction = safe_speed;
    }
  } else {
    vmax_junction = safe_speed;
  }

  previous_safe_speed = safe_speed;
  float vmax_junction_sqr = sq(vmax_junction);

  // END JERK LIMITING

  newBlock->entry_speed_max_sqr = vmax_junction_sqr;

  float allowable_speed_sqr = max_speed_allowed_sqr(-newBlock->acceleration, sq(float(MIN_FEEDRATE)), newBlock->distance);
  newBlock->entry_speed_sqr = min(vmax_junction_sqr, allowable_speed_sqr);
  SET_BIT( newBlock->flags, BIT_FLAG_NOMINAL, ( newBlock->nominal_speed_sqr <= allowable_speed_sqr ) );
  SET_BIT_ON( newBlock->flags, BIT_FLAG_RECALCULATE );

  previous_nominal_speed_sqr = newBlock->nominal_speed_sqr;
  for(ALL_MUSCLES(i)) {
    previous_speed[i] = current_speed[i];
  }
}

void Planner::segmentReport(Segment &new_seg) {
  Serial.print("seg:");  Serial.println((long)&new_seg,HEX);
  Serial.print("distance=");  Serial.println(new_seg.distance);
  Serial.print("nominal_speed=");  Serial.println(new_seg.nominal_speed_sqr);
  Serial.print("delta_units=");
  for (ALL_MUSCLES(i)) {
    if(i > 0) Serial.print(",");
    Serial.print(new_seg.a[i].delta_units);
  }
  Serial.println();
  Serial.print("nominal_rate=");  Serial.println(new_seg.nominal_rate);
  Serial.print("acceleration_steps_per_s2=");  Serial.println(new_seg.acceleration_steps_per_s2);
  Serial.print("acceleration=");  Serial.println(new_seg.acceleration);
  Serial.print("nominal_speed=");  Serial.println(new_seg.nominal_speed_sqr);
  Serial.print("entry_speed_max=");  Serial.println(new_seg.entry_speed_max_sqr);
  Serial.print("entry_speed=");  Serial.println(new_seg.entry_speed_sqr);
}

/**
   Translate the XYZ through the IK to get the number of motor steps and move the motors.
   Uses bresenham's line algorithm to move both motors
   @input pos NUM_AXIES floats describing destination coordinates
   @input new_feed_rate speed to travel along arc
   @input longest_distance must be >=0
*/
void Planner::addSegment(const float *const target_position, float fr_units_s, float longest_distance) {
  uint8_t next_buffer_head;
  Segment *newBlock = getNextFreeBlock(next_buffer_head);

  addSteps(newBlock,target_position,fr_units_s,longest_distance);

  // when should we accelerate and decelerate in this segment?
  //segment_update_trapezoid(newBlock, newBlock->entry_speed_sqr / newBlock->nominal_speed_sqr,(float)MIN_FEEDRATE / newBlock->nominal_speed_sqr);
  //segmentReport(new_seg);

  if(block_buffer_tail == block_buffer_head) {
    first_segment_delay = BLOCK_DELAY_FOR_1ST_MOVE;
  }
  block_buffer_head = next_buffer_head;

  recalculate();
  // describeAllSegments();

  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

/**
   Split long moves into sub-moves if needed.
   @input pos NUM_AXIES floats describing destination coordinates
   @input new_feed_rate speed to travel along arc
*/
void Planner::bufferLine(float *pos, float new_feed_rate_units) {
  // Remember the feed rate.  This value will be used whenever no feedrate is given in a command, so it MUST be saved
  // BEFORE the dial adjustment. otherwise the feedrate will slowly fall or climb as new commands are processed.
  desiredFeedRate = new_feed_rate_units;

#ifdef HAS_LCD
  // use LCD to adjust speed while drawing
  new_feed_rate_units *= (float)speed_adjust * 0.01f;
#endif

  // split up long lines to make them straighter
  float delta[NUM_AXIES];
  float startPos[NUM_AXIES];
  float lenSquared = 0;

  for (ALL_AXIES(i)) {
    startPos[i] = axies[i].pos;
    delta[i]    = pos[i] - startPos[i];
    lenSquared += sq(delta[i]);
  }

#if MACHINE_STYLE == POLARGRAPH
  if(delta[0] == 0 && delta[1] == 0) {
    // only moving Z, don't split the line.
    addSegment(pos, new_feed_rate_units, abs(delta[2]));
    return;
  }
#endif

  float len_units = sqrt(lenSquared);
  if(abs(len_units) < 0.000001f) return;

  const float seconds = len_units / new_feed_rate_units;
  uint16_t segments   = seconds * SEGMENTS_PER_SECOND;
  if(segments < 1) segments = 1;

#ifdef HAS_GRIPPER
  // if we have a gripper and only gripper is moving, don't split the movement.
  if(lenSquared == sq(delta[6])) {
    Serial.println("only t");
    segments = 1;
    Serial.print("seconds=");
    Serial.println(seconds);
    Serial.print("len_units=");
    Serial.println(len_units);
    Serial.print("new_feed_rate_units=");
    Serial.println(new_feed_rate_units);
  }
#endif

  const float inv_segments   = 1.0f / float(segments);
  const float segment_len_units = len_units * inv_segments;

  for (ALL_AXIES(i)) delta[i] *= inv_segments;

  while (--segments) {
    for (ALL_AXIES(i)) startPos[i] += delta[i];

    addSegment(startPos, new_feed_rate_units, segment_len_units);
  }

  // guarantee we stop exactly at the destination (no rounding errors).
  addSegment(pos, new_feed_rate_units, segment_len_units);

  //  Serial.print("P");  Serial.println(movesPlanned());
}

/**
   This method assumes the limits have already been checked.
   This method assumes the start and end radius match.
   This method assumes arcs are not >180 degrees (PI radians)
   @input cx center of circle x value
   @input cy center of circle y value
   @input destination point where movement ends
   @input dir - ARC_CW or ARC_CCW to control direction of arc
   @input new_feed_rate speed to travel along arc
*/
void Planner::bufferArc(float cx, float cy, float *destination, char clockwise, float new_feed_rate_units) {
  // get radius
  float dx = axies[0].pos - cx;
  float dy = axies[1].pos - cy;
  float sr = sqrt(dx * dx + dy * dy);

  // find angle of arc (sweep)
  float sa = atan3(dy, dx);
  float ea = atan3(destination[1] - cy, destination[0] - cx);
  float er = sqrt(dx * dx + dy * dy);

  float da = ea - sa;
  if(clockwise == ARC_CW && da < 0)
    ea += 2 * PI;
  else if(clockwise == ARC_CCW && da > 0)
    sa += 2 * PI;
  da       = ea - sa;
  float dr = er - sr;

  // get length of arc
  // float circ=PI*2.0*radius;
  // float len=theta*circ/(PI*2.0);
  // simplifies to
  float len1 = abs(da) * sr;
  float len  = sqrt(len1 * len1 + dr * dr);  // mm

  int i, segments = ceil(len);

  float n[NUM_AXIES], scale;
  float a, r;
#if NUM_AXIES > 2
  float sz = axies[2].pos;
  float z  = destination[2];
#endif

  for (i = 0; i <= segments; ++i) {
    // interpolate around the arc
    scale = ((float)i) / ((float)segments);

    a = (da * scale) + sa;
    r = (dr * scale) + sr;

    n[0] = cx + cos(a) * r;
    n[1] = cy + sin(a) * r;
#if NUM_AXIES > 2
    n[2] = (z - sz) * scale + sz;
#endif
    // send it to the planner
    bufferLine(n, new_feed_rate_units);
  }
}

void Planner::estop() {
  // clear segment buffer
  block_buffer_nonbusy = block_buffer_planned = block_buffer_head = block_buffer_tail;
}

#if MACHINE_STYLE == POLARGRAPH && defined(DYNAMIC_ACCELERATION)
float Planner::limitPolargraphAcceleration(const float *target_position,const float *oldP,float maxAcceleration) {
  // Adjust the maximum acceleration based on the plotter position to reduce wobble at the bottom of the picture.
  // We only consider the XY plane.
  // Special thanks to https://www.reddit.com/user/zebediah49 for his math help.

  // if T is your target direction unit vector,
  float Tx = target_position[0] - oldP[0];
  float Ty = target_position[1] - oldP[1];
  float Rlen = sq(Tx) + sq(Ty);  // always >=0
  if(Rlen > 0) {
    // normal vectors pointing from plotter to motor
    float R1x = axies[0].limitMin - oldP[0];  // to left
    float R1y = axies[1].limitMax - oldP[1];  // to top
    float Rlen1 = 1.0 / sqrt(sq(R1x) + sq(R1y));//old_seg.a[0].step_count * UNITS_PER_STEP;
    R1x *= Rlen1;
    R1y *= Rlen1;

    float R2x = axies[0].limitMax - oldP[0];  // to right
    float R2y = axies[1].limitMax - oldP[1];  // to top
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
    if(c1 > 0 && c2 > 0) {
      cT = (c1 < c2) ? c1 : c2;
    } else if(c1 > 0) {
      cT = c1;
    } else if(c2 > 0) {
      cT = c2;
    }

    // The maximum acceleration is given by cT if cT>0
    if(cT > 0) {
      maxAcceleration = max(min(maxAcceleration, cT), (float)MIN_ACCELERATION);
    }
  }
  return maxAcceleration;
}
#endif
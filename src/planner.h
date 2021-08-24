#pragma once

//------------------------------------------------------------------------------

#ifndef MAX_SEGMENTS
#  define MAX_SEGMENTS (32)  // number of line segments to buffer ahead. must be a power of two.
#endif

#define SEGMOD(x) ((x) & (MAX_SEGMENTS - 1))


// uncomment this to slow the machine and smooth movement if the segment buffer is running low.
// if the segment buffer runs empty the machine will be forced to stop.  if the machine slows just enough
// for the buffer to keep filling up then the machine will never come to a complete stop.  end result smoother movement and maybe shorter drawing time.
#define BUFFER_EMPTY_SLOWDOWN
#ifndef DEFAULT_MIN_SEGMENT_TIME_US
#  define DEFAULT_MIN_SEGMENT_TIME_US (25000)
#endif

// if a segment added to the buffer is less than this many motor steps, roll it into the next move.
#ifndef MIN_STEPS_PER_SEGMENT
#define MIN_STEPS_PER_SEGMENT 6
#endif

#define MINIMUM_PLANNER_SPEED 0.05  // (mm/s)

#define BLOCK_DELAY_FOR_1ST_MOVE 100
#define MIN_STEP_RATE            120
#define GRAVITYmag               (9800.0)

//#define HAS_CLASSIC_JERK
#if !defined(HAS_CLASSIC_JERK)
#define HAS_JUNCTION_DEVIATION 1
#define JUNCTION_DEVIATION_UNITS 0.05
#define JUNCTION_DEVIATION_MAX 0.5
#define JUNCTION_DEVIATION_MIN 0.001
#define JD_HANDLE_SMALL_SEGMENTS
#endif
#if !defined(HAS_CLASSIC_JERK) && !defined(HAS_JUNCTION_DEVIATION)
#define DOT_PRODUCT_JERK
#endif

//------------------------------------------------------------------------------

class Muscle {
public:
  int32_t step_count;   // current motor position, in steps.
  uint32_t absdelta;
#ifdef HAS_POSITION_SENSORS
  float expectedPosition;
  float positionStart;
  float positionEnd;
#endif
};


enum BlockFlagBits : uint8_t { 
    BIT_FLAG_NOMINAL,
    BIT_FLAG_RECALCULATE
};

enum BlockFlagMask : uint8_t {
  BLOCK_FLAG_NOMINAL     = _BV(BIT_FLAG_NOMINAL),
  BLOCK_FLAG_RECALCULATE = _BV(BIT_FLAG_RECALCULATE),
};


class Segment {
public:
  Muscle a[NUM_MUSCLES];
  uint16_t dir;

  float distance;         // units
  float nominal_speed_sqr;    // units/s
  float entry_speed_sqr;      // units/s
  float entry_speed_max_sqr;  // units/s
  float acceleration;     // units/sec^2

  uint32_t steps_total;  // steps
  uint32_t steps_taken;  // steps
  uint32_t accel_until;  // steps
  uint32_t decel_after;  // steps

  uint32_t nominal_rate;               // steps/s
  uint32_t initial_rate;               // steps/s
  uint32_t final_rate;                 // steps/s
  uint32_t acceleration_steps_per_s2;  // steps/s^2
  
#if defined(S_CURVE_ACCELERATION)
  uint32_t cruise_rate,                   // The actual cruise rate to use, between end of the acceleration phase and start of deceleration phase
            acceleration_time,             // Acceleration time and deceleration time in STEP timer counts
            deceleration_time,
            acceleration_time_inverse,     // Inverse of acceleration and deceleration periods, expressed as integer. Scale depends on CPU being used
            deceleration_time_inverse;
#else
  uint32_t acceleration_rate;  // ?
#endif

  uint8_t flags;
};


class Planner {
  public:
  static Segment blockBuffer[MAX_SEGMENTS];
  static volatile int block_buffer_head, 
                      block_buffer_nonbusy,
                      block_buffer_planned,
                      block_buffer_tail;
  static int first_segment_delay;

  static float previous_nominal_speed_sqr;
  static float previous_safe_speed;
  static float previous_speed[NUM_MUSCLES];
  static float prev_unit_vec[NUM_AXIES];
  
#ifdef HAS_JUNCTION_DEVIATION
  static float junction_deviation;
#endif

  FORCE_INLINE static uint8_t movesPlanned() {
    return SEGMOD(block_buffer_head - block_buffer_tail);
  }

  FORCE_INLINE static uint8_t movesFree() {
      return MAX_SEGMENTS - 1 - movesPlanned();
  }

  FORCE_INLINE static uint8_t movesPlannedNotBusy() {
    return SEGMOD(block_buffer_head - block_buffer_nonbusy);
  }

  FORCE_INLINE static int getNextBlock(int i) {
    return SEGMOD(i + 1);
  }

  FORCE_INLINE static int getPrevBlock(int i) {
    return SEGMOD(i - 1);
  }

  FORCE_INLINE void releaseCurrentBlock() {
    block_buffer_tail = getNextBlock(block_buffer_tail);
  }

  FORCE_INLINE Segment *getCurrentBlock() {
    if (block_buffer_tail == block_buffer_head) return NULL;

    if (first_segment_delay > 0) {
      --first_segment_delay;
      if (movesPlanned() < 3 && first_segment_delay) return NULL;
      first_segment_delay=0;
    }

    Segment *block = &blockBuffer[block_buffer_tail];

    if( TEST(block->flags,BIT_FLAG_RECALCULATE) ) return NULL;  // wait, not ready

    block_buffer_nonbusy = getNextBlock(block_buffer_tail);
    if (block_buffer_tail == block_buffer_planned) {
      block_buffer_planned = block_buffer_nonbusy;
    }

    return block;
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

  static void zeroSpeeds();
  static void wait_for_empty_segment_buffer();

  static bool populateBlock(Segment *newBlock,const float *const target_position, float fr_units_s, float longest_distance);
  static void addSegment(const float *const target_position, float fr_units_s, float millimeters);
  static void bufferLine(float *pos, float new_feed_rate_units);
  static void bufferArc(float cx, float cy, float *destination, char clockwise, float new_feed_rate_units);

  static float max_speed_allowed_sqr(const float &acc, const float &target_velocity, const float &distance);

  static void recalculate_reverse_kernel(Segment *const current, const Segment *next);
  static void reversePass();
  static void recalculate_forward_kernel(const Segment *prev, Segment *const current,uint8_t block_index);
  static void forwardPass();
  static void recalculate();

  static float estimate_acceleration_distance(const float &initial_rate, const float &target_rate, const float &accel);
  static int intersection_distance(const float &start_rate, const float &end_rate, const float &accel, const float &distance);

  static void calculate_trapezoid_for_block(Segment *s, const float &entry_factor, const float &exit_factor);
  static void recalculate_trapezoids();

  static void describeAllSegments();
  static void segmentReport(Segment &new_seg);

  static void estop();

#  if MACHINE_STYLE == POLARGRAPH && defined(DYNAMIC_ACCELERATION)
  static float limitPolargraphAcceleration(const float *target_position,const float *oldP,float maxAcceleration);
#  endif

  FORCE_INLINE static float normalize_junction_vector(float *v) {
    float m=0;
    for(ALL_MOTORS(i)) m += sq(v[i]);
    float im = 1.0f/sqrtf(m);
    for(ALL_MOTORS(i)) v[i]*=im;
    return m;
  }

  FORCE_INLINE static float limit_value_by_axis_maximum(const float max_value, float *unit_vec,const float max_acceleration) {
    float limit_value = max_value;
    for(ALL_MOTORS(i)) {
      if(unit_vec[i]) {
        if(limit_value * abs(unit_vec[i]) > max_acceleration)
          limit_value = abs( max_acceleration / unit_vec[i] );
      }
    }
    return limit_value;
  }


#if defined(S_CURVE_ACCELERATION)
  // Calculate the speed reached given initial speed, acceleration and distance
  static float final_speed(const float initial_velocity, const float accel, const float distance) {
    return sqrtf(sq(initial_velocity) + 2 * accel * distance);
  }
#endif

#ifdef HAS_JUNCTION_DEVIATION
  static float junctionDeviation(Segment *newBlock,float *delta,int movesQueued,float inverseCartesianDistance,float max_acceleration);
#endif
#ifdef HAS_CLASSIC_JERK
  static float classicJerk(Segment *newBlock,float *current_speed,int movesQueued);
#endif
};

extern Planner planner;
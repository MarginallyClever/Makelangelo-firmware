#pragma once
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == SIXI

#define MOTHERBOARD BOARD_SIXI_MEGA  // forced

#define MACHINE_STYLE_NAME           "SIXI"
#define MACHINE_HARDWARE_VERSION     6  // yellow sixi 2019

#define STEP_DELAY           (50)  // delay between steps, in milliseconds, when doing fixed tasks like homing

// plan long moves as a set of submoves to increase accuracy.  Uncomment to turn this off.
#define SEGMENTS_PER_SECOND  (40)

#define NUM_AXIES            (7)
#define NUM_MOTORS           (6)
#define NUM_TOOLS            (1)
#define NUM_SENSORS          (6)

#define NUM_SERVOS           (1)
//#define HAS_GRIPPER  // uncomment this to use a gripper

#define MAX_FEEDRATE         (120.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (0)
#define DEFAULT_FEEDRATE     (80.0)

#define MAX_ACCELERATION     (50.0)
#define MIN_ACCELERATION     (0)
#define DEFAULT_ACCELERATION (25.0)

#define MAX_JERK             (3.0)

#define MAX_SEGMENTS         (16)  // override the default to save RAM
#define DEGREES_PER_STEP     (1.8)
#define MICROSTEPS           (1.0)

//#define HAS_LCD
#define HAS_SD
#define MOTHERBOARD BOARD_SIXI_MEGA  // sixi only supports one motherboard right now


#define MOTOR_STEPS_PER_TURN          (200.0)  // motor full steps * microstepping setting

#define NEMA17_CYCLOID_GEARBOX_RATIO        (20.0)
#define NEMA23_CYCLOID_GEARBOX_RATIO_ELBOW  (35.0)
#define NEMA23_CYCLOID_GEARBOX_RATIO_ANCHOR (30.0)
#define NEMA24_CYCLOID_GEARBOX_RATIO        (40.0)

#define DM322T_MICROSTEP              (2.0)

#define ELBOW_DOWNGEAR_RATIO          (30.0/20.0)
#define NEMA17_RATIO                  (DM322T_MICROSTEP*NEMA17_CYCLOID_GEARBOX_RATIO*ELBOW_DOWNGEAR_RATIO)
#define NEMA23_RATIO_ELBOW            (NEMA23_CYCLOID_GEARBOX_RATIO_ELBOW)
#define NEMA23_RATIO_ANCHOR           (NEMA23_CYCLOID_GEARBOX_RATIO_ANCHOR)
#define NEMA24_RATIO                  (NEMA24_CYCLOID_GEARBOX_RATIO)

// Motors are numbered 0 (base) to 5 (hand)
#define MOTOR_0_STEPS_PER_TURN    (MOTOR_STEPS_PER_TURN*NEMA23_RATIO_ANCHOR)  // anchor
#define MOTOR_1_STEPS_PER_TURN    (MOTOR_STEPS_PER_TURN*NEMA24_RATIO)  // shoulder
#define MOTOR_2_STEPS_PER_TURN    (MOTOR_STEPS_PER_TURN*NEMA23_RATIO_ELBOW)  // elbow
#define MOTOR_3_STEPS_PER_TURN    (MOTOR_STEPS_PER_TURN*NEMA17_RATIO)  // ulna
#define MOTOR_4_STEPS_PER_TURN    (MOTOR_STEPS_PER_TURN*NEMA17_RATIO)  // wrist
#define MOTOR_5_STEPS_PER_TURN    (MOTOR_STEPS_PER_TURN*NEMA17_RATIO)  // hand

#define DEGREES_PER_STEP_0 (360.0/MOTOR_0_STEPS_PER_TURN)
#define DEGREES_PER_STEP_1 (360.0/MOTOR_1_STEPS_PER_TURN)
#define DEGREES_PER_STEP_2 (360.0/MOTOR_2_STEPS_PER_TURN)
#define DEGREES_PER_STEP_3 (360.0/MOTOR_3_STEPS_PER_TURN)
#define DEGREES_PER_STEP_4 (360.0/MOTOR_4_STEPS_PER_TURN)
#define DEGREES_PER_STEP_5 (360.0/MOTOR_5_STEPS_PER_TURN)
#define MM_PER_STEP        1

#define POSITION_EPSILON (1.9) // degrees

// step signal start
#define START0 LOW
#define START1 LOW
#define START2 LOW
#define START3 HIGH
#define START4 HIGH
#define START5 HIGH

// step signal end
#define END0 HIGH
#define END1 HIGH
#define END2 HIGH
#define END3 LOW
#define END4 LOW
#define END5 LOW

// DH parameter table (kinematics)
#define DH_0_THETA 0
#define DH_0_ALPHA -90
#define DH_0_D     19.745
#define DH_0_R     0
#define DH_0_MAX   120
#define DH_0_MIN   -120

#define DH_1_THETA -90
#define DH_1_ALPHA 0
#define DH_1_D     0
#define DH_1_R     35.796
#define DH_1_MAX   0
#define DH_1_MIN   -170

#define DH_2_THETA 0
#define DH_2_ALPHA -90
#define DH_2_D     0
#define DH_2_R     6.426
#define DH_2_MAX   86
#define DH_2_MIN   -83.369

#define DH_3_THETA 0
#define DH_3_ALPHA 90
#define DH_3_D     38.705
#define DH_3_R     0
#define DH_3_MAX   175
#define DH_3_MIN   -175

#define DH_4_THETA 0
#define DH_4_ALPHA -90
#define DH_4_D     0
#define DH_4_R     0
#define DH_4_MAX   120
#define DH_4_MIN   -120

#define DH_5_THETA 0
#define DH_5_ALPHA 0
#define DH_5_D     5.795
#define DH_5_R     0
#define DH_5_MAX   170
#define DH_5_MIN   -170

// behaviour flags
#define POSITION_ERROR_FLAG_CONTINUOUS   (0)  // report position (d17) continuously?
#define POSITION_ERROR_FLAG_ERROR        (1)  // has error occurred?
#define POSITION_ERROR_FLAG_ESTOP        (2)  // emergency stop!
#define POSITION_ERROR_FLAG_CHECKLIMIT   (3)  // check limits and throw error if needed (normally only disabled to drive the robot back inside limits)

// SENSORS
#define REPORT_ANGLES_CONTINUOUSLY (TEST(sensorManager.positionErrorFlags,POSITION_ERROR_FLAG_CONTINUOUS))
#define TEST_LIMITS                (TEST(sensorManager.positionErrorFlags,POSITION_ERROR_FLAG_CHECKLIMIT))
#define OUT_OF_BOUNDS              (TEST(sensorManager.positionErrorFlags,POSITION_ERROR_FLAG_ERROR))

// use in for(ALL_SENSORS(i)) { //i will be rising
#define ALL_SENSORS(NN) int NN=0;NN<NUM_SENSORS;++NN

// sensor bits, flags, and masks
#define BOTTOM_14_MASK       (0x3FFF)
#define SENSOR_TOTAL_BITS    (16)
#define SENSOR_DATA_BITS     (15)
#define SENSOR_ANGLE_BITS    (14)
#define SENSOR_ANGLE_PER_BIT (360.0/(float)((long)1<<SENSOR_ANGLE_BITS))  // 0.02197265625


#include <Arduino.h>  // for uint8_t

class SensorAS5147 {
public:
  uint8_t pin_CSEL;
  uint8_t pin_CLK;
  uint8_t pin_MISO;
  uint8_t pin_MOSI;
  float angle; // current reading after adjustment
  float angleHome;  // sensor raw angle value at home position.  reading - this = 0.

  void start();
  
  /**
   * See https://ams.com/documents/20143/36005/AS5147_DS000307_2-00.pdf
   * @param result where to store the returned value.  may be changed even if method fails.
   * @return 0 on fail, 1 on success.
   */
  bool getRawValue(uint16_t &result);
};

class SensorManager {
public:
  SensorAS5147 sensors[NUM_SENSORS];
  uint8_t positionErrorFlags;

  void resetAll();
  
  void updateAll();
  void setup();
  
  /**
   * @param rawValue 16 bit value from as4157 sensor, including parity and EF bit
   * @return degrees calculated from bottom 14 bits.
   */
  inline float extractAngleFromRawValue(uint16_t rawValue) {
    return (float)(rawValue & BOTTOM_14_MASK) * 360.0 / (float)(1<<SENSOR_ANGLE_BITS);
  }
};

extern SensorManager sensorManager;

extern void sixiDemo();
extern void sixiSetup();

extern void gripperUpdate(float currentGripperCmd);


#endif  // #ifdef SIXI

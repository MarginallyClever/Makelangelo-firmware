//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#include "configure.h"

#if MACHINE_STYLE == SIXI3

#  include <Arduino.h>
/**
 * Inverse Kinematics turns cartesian coordinates into step counts from each motor
 * @param cartesian array of cartesian coordinates
 * @param motorStepArray number of steps per motor
 */
void IK(const float *const cartesian, int32_t *motorStepArray) {
  for (ALL_AXIES(i)) {
    motorStepArray[i] = lround(cartesian[i]);
  }
#if NUM_SERVOS>0
  // servo
  motorStepArray[NUM_MOTORS] = lround(cartesian[NUM_MOTORS]);
#endif
  for(ALL_MUSCLES(i)) {
    motorStepArray[i] *= motor_spu[i];
  }

}

/**
 * Forward Kinematics - turns step counts into XY coordinates
 * @param motorStepArray a measure of each belt to that plotter position
 * @param axies the resulting cartesian coordinate
 * @return 0 if no problem, 1 on failure.
 */
int FK(int32_t *motorStepArray, float *axies) {
  for (ALL_AXIES(i)) {
    axies[i] = motorStepArray[i] * UNITS_PER_STEP;
  }
#if NUM_SERVOS>0
  // servo
  axies[NUM_MOTORS] = motorStepArray[NUM_MOTORS];
#endif
  return 0;
}

void robot_findHome() {
  planner.wait_for_empty_segment_buffer();
  Stepper::engage();

  hal_timer_t stepDelay = findStepDelay();

  MYSERIAL1.println(F("Finding..."));

  uint8_t hits;
  // back up until all switches are hit
  do {
    hits = 0;
    // for each stepper,
    for (ALL_MOTORS(i)) {
      digitalWrite(motors[i].dir_pin, HIGH);
      // if this switch hasn't been hit yet
      if (digitalRead(motors[i].limit_switch_pin) == HIGH) {
        // move "down"
        MYSERIAL1.print('|');
        digitalWrite(motors[i].step_pin, HIGH);
        digitalWrite(motors[i].step_pin, LOW);
      } else {
        ++hits;
        MYSERIAL1.print('*');
      }
    }
    SERIAL_EOL();
    pause(stepDelay);
  } while (hits < NUM_MOTORS);
  MYSERIAL1.println(F("Found."));

  float zeros[6] = { 0, 0, 0, 0, 0, 0 };
  Planner::teleport(zeros);
}

void robot_setup() {
}

void factory_reset() {
  for(ALL_MOTORS(i)) {
    motor_spu[i]=STEPS_PER_UNIT;
  }
  motor_spu[NUM_MOTORS]=1;
  
  for (ALL_MUSCLES(i)) {
    max_jerk[i] = MAX_JERK_DEFAULT;
    max_step_rate_s[i] = MAX_STEP_RATE_DEFAULT;
  }

  // if you accidentally upload m3 firmware to an m5 then upload it ONCE with this line uncommented.
  float limits[NUM_AXIES * 2];
  for(ALL_AXIES(i)) {
    limits[i*2+0] = -360.0;
    limits[i*2+1] =  360.0;
  }

  eepromManager.adjustLimits(limits);
  eepromManager.saveAll();
}

#endif  // SIXI3

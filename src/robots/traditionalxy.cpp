//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#include "configure.h"

#if MACHINE_STYLE == TRADITIONALXY

#  include <Arduino.h>
/**
 * Inverse Kinematics turns XY coordinates into step counts from each motor
 * @param x cartesian coordinate
 * @param y cartesian coordinate
 * @param motorStepArray a measure of each belt to that plotter position
 */
void IK(const float *const cartesian, int32_t *motorStepArray) {
  float x = cartesian[0];
  float y = cartesian[1];
  float z = cartesian[2];

  motorStepArray[0] = lround(x * motor_spu[0]);
  motorStepArray[1] = lround(y * motor_spu[1]);

  motorStepArray[NUM_MOTORS] = z * motor_spu[2];
}

/**
 * Forward Kinematics - turns step counts into XY coordinates
 * @param motorStepArray a measure of each belt to that plotter position
 * @param axies the resulting cartesian coordinate
 * @return 0 if no problem, 1 on failure.
 */
int FK(int32_t *motorStepArray, float *axies) {
  axies[0] = motorStepArray[0] * UNITS_PER_STEP_X;
  axies[1] = motorStepArray[1] * UNITS_PER_STEP_Y;
  axies[2] = motorStepArray[NUM_MOTORS];
}

void robot_findHome() {
  planner.wait_for_empty_segment_buffer();
  Stepper::engage();

  hal_timer_t stepDelay = findStepDelay();

  MYSERIAL1.println(F("Finding..."));

  uint8_t i, hits;
  // back up until all switches are hit
  do {
    hits = 0;
    // for each stepper,
    for (ALL_MOTORS(i)) {
      digitalWrite(motors[i].dir_pin, HIGH);
      // if this switch hasn't been hit yet
      if (digitalRead(motors[i].limit_switch_pin) == HIGH) {
        digitalWrite(motors[i].step_pin, HIGH);
        digitalWrite(motors[i].step_pin, LOW);
      } else {
        ++hits;
      }
    }
    SERIAL_EOL();
    pause(stepDelay);
  } while (hits < NUM_MOTORS);
  MYSERIAL1.println(F("Found."));

  float zeros[2] = { 0, 0 };
  teleport(zeros);
}

void robot_setup() {}

#endif

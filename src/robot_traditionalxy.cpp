//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#include "configure.h"

#if MACHINE_STYLE == TRADITIONALXY

#include <Arduino.h>
/**
 * Inverse Kinematics turns XY coordinates into step counts from each motor
 * @param x cartesian coordinate
 * @param y cartesian coordinate
 * @param motorStepArray a measure of each belt to that plotter position
 */
void IK(const float *const cartesian, long *motorStepArray) {
  float x = cartesian[0];
  float y = cartesian[1];
  float z = cartesian[2];
  
  motorStepArray[0] = lround(x / MM_PER_STEP_X);
  motorStepArray[1] = lround(y / MM_PER_STEP_Y);

  motorStepArray[NUM_MOTORS] = z;
}


/** 
 * Forward Kinematics - turns step counts into XY coordinates
 * @param motorStepArray a measure of each belt to that plotter position
 * @param axies the resulting cartesian coordinate
 * @return 0 if no problem, 1 on failure.
 */
int FK(long *motorStepArray,float *axies) {
  axies[0] = motorStepArray[0] * MM_PER_STEP_X;
  axies[1] = motorStepArray[1] * MM_PER_STEP_Y;
  axies[2] = motorStepArray[NUM_MOTORS];
}



void robot_findHome() {
  wait_for_empty_segment_buffer();
  motor_engage();

  findStepDelay();

  Serial.println(F("Finding..."));

  uint8_t i, hits;
  // back up until all switches are hit
  do {
    hits = 0;
    // for each stepper,
    for (ALL_MOTORS(i)) {
      digitalWrite(motors[i].dir_pin, HIGH);
      // if this switch hasn't been hit yet
      if (digitalRead(motors[i].limit_switch_pin) == HIGH) {
        // move "down"
        Serial.print('|');
        digitalWrite(motors[i].step_pin, HIGH);
        digitalWrite(motors[i].step_pin, LOW);
      } else {
        ++hits;
        Serial.print('*');
      }
    }
    Serial.println();
    pause(step_delay);
  } while (hits < NUM_MOTORS);
  Serial.println(F("Found."));
  
  float zeros[2] = {0, 0};
  teleport(zeros);
}


void robot_setup() {
}

#endif

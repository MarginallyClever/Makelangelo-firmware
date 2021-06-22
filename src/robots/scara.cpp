//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#include "configure.h"

#if MACHINE_STYLE == SCARA

#  include <Arduino.h>

// use law of cosines property to find one interior angle in a triangle.  c=arccos((aa+bb-cc)/(2ab)).
float lawOfCosines(float a, float b, float c) {
  float numerator   = sq(a) + sq(b) - sq(c);
  float denominator = 2.0 * a * b;
  if (denominator == 0) return 0;
  return acos(numerator / denominator);
}

/**
 * Inverse Kinematics turns XY coordinates into step counts from each motor
 * @param x cartesian coordinate
 * @param y cartesian coordinate
 * @param motorStepArray a measure of each belt to that plotter position
 */
void IK(const float *const cartesian, int32_t *motorStepArray) {
  // see https://appliedgo.net/roboticarm/
  float x = cartesian[0];
  float y = cartesian[1];
  float z = cartesian[2];

  // BICEP_LENGTH_MM and FOREARM_LENGTH_MM are defined in robot_scara.h.
  // TODO save the numbers in EEPROM so they can be tweaked without a recompile?

  // from cartesian x,y we can get c, the distance from origin to x,y.
  // use law of cosines to
  float c = sqrtf(sq(x) + sq(y));
  // then law of cosines will give us the elbow angle
  float elbowAngle = lawOfCosines(BICEP_LENGTH_MM, FOREARM_LENGTH_MM, c);

  // shoulder angle is made of two parts: the interior corner inside the
  float shoulderAngle = atan2(y, x) + lawOfCosines(c, BICEP_LENGTH_MM, FOREARM_LENGTH_MM);

  // angles are in radians.  we need degrees
  motorStepArray[0] = lround(DEGREES(shoulderAngle));
  motorStepArray[1] = lround(DEGREES(elbowAngle));
  motorStepArray[NUM_MOTORS] = z;

  motorStepArray[0] *= motor_spu[0];
  motorStepArray[1] *= motor_spu[1];
  motorStepArray[2] *= motor_spu[2];
}

/**
 * Forward Kinematics - turns step counts into XY coordinates
 * @param motorStepArray a measure of each belt to that plotter position
 * @param axies the resulting cartesian coordinate
 * @return 0 if no problem, 1 on failure.
 */
int FK(int32_t *motorStepArray, float *axies) {
  float a = RADIANS(motorStepArray[0] * MICROSTEP_PER_DEGREE);
  float b = RADIANS(motorStepArray[1] * MICROSTEP_PER_DEGREE);

  axies[0] = cos(a) * BICEP_LENGTH_MM + cos(a + b) * FOREARM_LENGTH_MM;
  axies[1] = sin(a) * BICEP_LENGTH_MM + sin(a + b) * FOREARM_LENGTH_MM;
  axies[2] = motorStepArray[NUM_MOTORS];
}

void robot_findHome() {
  planner.wait_for_empty_segment_buffer();
  Stepper::engage();

  hal_timer_t stepDelay = findStepDelay();

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
    pause(stepDelay);
  } while (hits < NUM_MOTORS);
  Serial.println(F("Found."));

  float zeros[2] = { 0, 0 };
  teleport(zeros);
}

void robot_setup() {}

#endif  // SCARA

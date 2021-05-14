//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#include "configure.h"

#if MACHINE_STYLE == STEWART_LINEAR

#  include "vector3.h"

#  define NUM_ARMS (6)

void stewartDemo() {
  float pos[NUM_AXIES] = { 0, 0, 0, 0, 0, 0 };
  int i, j;

  // linear moves
  for (j = 0; j < 3; ++j) {
    for (i = 0; i < 5; ++i) {
      pos[j] = 2;
      lineSafe(pos, feed_rate);
      pos[j] = -2;
      lineSafe(pos, feed_rate);
    }
    pos[j] = 0;
    lineSafe(pos, feed_rate);
  }

  // tilting
  for (j = 4; j < 6; ++j) {
    for (i = 0; i < 5; ++i) {
      pos[j] = 10;
      lineSafe(pos, feed_rate);
      pos[j] = -10;
      lineSafe(pos, feed_rate);
    }
    pos[j] = 0;
    lineSafe(pos, feed_rate);
  }

  // combos
  for (j = 0; j < 3; ++j) {
    for (i = 0; i < 360 * 5; i += 10) {
      pos[j]           = 2 * sin(((float)i / 180.0) * PI);
      pos[(j + 1) % 3] = 2 * cos(((float)i / 180.0) * PI);
      lineSafe(pos, feed_rate);
    }
    pos[j]           = 0;
    pos[(j + 1) % 3] = 0;
    lineSafe(pos, feed_rate);
  }

  // combos rotation
  for (j = 4; j < 6; ++j) {
    for (i = 0; i < 360 * 5; i += 10) {
      pos[j]                     = 10.0 * sin(((float)i / 180.0) * PI);
      pos[3 + ((j - 3 + 1) % 3)] = 10.0 * cos(((float)i / 180.0) * PI);
      lineSafe(pos, feed_rate);
    }
    pos[j]                     = 0;
    pos[3 + ((j - 3 + 1) % 3)] = 0;
    lineSafe(pos, feed_rate);
  }

  // combos random
  for (j = 0; j < 50; ++j) {
    pos[0] = 2 * cos(((float)random(360) / 180.0) * PI);
    pos[1] = 2 * cos(((float)random(360) / 180.0) * PI);
    pos[2] = 1.5 * cos(((float)random(360) / 180.0) * PI);
    pos[3] = 10.0 * cos(((float)random(360) / 180.0) * PI);
    pos[4] = 10.0 * cos(((float)random(360) / 180.0) * PI);
    pos[5] = 10.0 * cos(((float)random(360) / 180.0) * PI);
    lineSafe(pos, feed_rate);
  }
  for (i = 0; i < NUM_AXIES; ++i) { pos[i] = 0; }
  lineSafe(pos, feed_rate);
}

/**
   Inverse Kinematics turns XY coordinates into step counts from each motor
   @param axies the cartesian coordinate
   @param motorStepArray a measure of each belt to that plotter position
*/
void IK(const float *const cartesian, long *motorStepArray) {
  for(int i = 0; i < NUM_ARMS; ++i) {
     motorStepArray[i] = cartesian[i] * STEPS_PER_UNIT;
  }
}

/**
   Forward Kinematics - turns step counts into XY coordinates
   @param motorStepArray a measure of each belt to that plotter position
   @param axies the resulting cartesian coordinate
   @return 0 if no problem, 1 on failure.
*/
int FK(long *motorStepArray, float *axies) {
  return 0;
}

void robot_setup() {}

void robot_findHome() {
  wait_for_empty_segment_buffer();
  motor_engage();

#ifdef HAS_TMC2130
  delay(500);
  digitalWrite(MOTOR_0_DIR_PIN, STEPPER_DIR_HIGH);
  digitalWrite(MOTOR_1_DIR_PIN, STEPPER_DIR_HIGH);

  tmc2130_motor_home();
#else
  findStepDelay();

  Serial.println(F("Finding..."));

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

#endif

  // set robot to home position
  float zeros[6] = { 0, 0, 0, 0, 0, 0 };
  teleport(zeros);
}


void factory_reset() {
  for(ALL_MOTORS(i)) {
    motor_spu[i]=STEPS_PER_UNIT;
  }
  eepromManager.saveSPU();
}

#endif

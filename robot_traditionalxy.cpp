//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#include "configure.h"
#include "robot_traditionalxy.h"

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

}


void robot_setup() {
}

#endif

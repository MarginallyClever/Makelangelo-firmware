//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#include "configure.h"
#include "robot_corexy.h"

#if MACHINE_STYLE == COREXY

/**
 * Inverse Kinematics turns XY coordinates into lengths L1,L2
 * @param axies the cartesian coordinate
 * @param motorStepArray a measure of each belt to that plotter position
 */
void IK(const float *const axies, long *motorStepArray) {
  float x = axies[0];
  float y = axies[1];
  float z = axies[2];

  motorStepArray[0] = lround((x+y) / MM_PER_STEP);
  motorStepArray[1] = lround((x-y) / MM_PER_STEP);

  motorStepArray[NUM_MOTORS] = z;
}


/** 
 * Forward Kinematics - turns L1,L2 lengths into XY coordinates
 * @param motorStepArray a measure of each belt to that plotter position
 * @param axies the resulting cartesian coordinate
 * @return 0 if no problem, 1 on failure.
 */
int FK(long *motorStepArray,float *axies) {
  float a = motorStepArray[0] * MM_PER_STEP;
  float b = motorStepArray[1] * MM_PER_STEP;

  float x = (float)( a + b ) / 2.0;
  float y = x - (float)b;
  
  axies[0]=x;
  axies[1]=y;
  axies[2]=motorStepArray[NUM_MOTORS];
  return 0;
}


void robot_findHome() {

}


void robot_setup() {
}


#endif

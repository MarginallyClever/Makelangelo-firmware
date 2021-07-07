//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#include "configure.h"

#if MACHINE_STYLE == SKYCAM

/**
 * Inverse Kinematics turns XY coordinates into lengths of belt from each motor
 * @param axies the cartesian coordinates
 * @param motorStepArray a measure of each belt to that plotter position
 */
void IK(const float *const cartesian, int32_t *motorStepArray) {
  float x = cartesian[0];
  float y = cartesian[1];
  float z = cartesian[2];

  float limit_xmax = axies[0].limitMax;
  float limit_xmin = axies[0].limitMin;
  float limit_ymax = axies[1].limitMax;
  float limit_ymin = axies[1].limitMin;

  float dy, dx, dz;

  dz = sq(z);

  dy                = sq(abs(y - limit_ymax) - SKYCAM_COMPENSATION);
  dx                = sq(abs(x - limit_xmin) - SKYCAM_COMPENSATION);
  motorStepArray[0] = lround(sqrtf(dx + dy + dz));  // M0 (top left)
  dy                = sq(abs(y - limit_ymax) - SKYCAM_COMPENSATION);
  dx                = sq(abs(x - limit_xmax) - SKYCAM_COMPENSATION);
  motorStepArray[1] = lround(sqrtf(dx + dy + dz));  // M1 (top right)
  dy                = sq(abs(y - limit_ymin) - SKYCAM_COMPENSATION);
  dx                = sq(abs(x - limit_xmin) - SKYCAM_COMPENSATION);
  motorStepArray[2] = lround(sqrtf(dx + dy + dz));  // M2 (bottom left)
  dy                = sq(abs(y - limit_ymin) - SKYCAM_COMPENSATION);
  dx                = sq(abs(x - limit_xmax) - SKYCAM_COMPENSATION);
  motorStepArray[3] = lround(sqrtf(dx + dy + dz));  // M3 (bottom right)

  for(ALL_MUSCLES(i)) {
    motorStepArray[i] *= motor_spu[i];
  }

}

/**
 * Forward Kinematics - turns L1,L2 lengths into XY coordinates
 * @param motorStepArray a measure of each belt to that plotter position
 * @param axies the resulting cartesian coordinate
 * @return 0 if no problem, 1 on failure.
 */
int FK(int32_t *motorStepArray, float *axies) {
  MYSERIAL1.println(F("UnimplmentedException"));
  return 1;
}

void robot_findHome() {}

void robot_setup() {}

#endif

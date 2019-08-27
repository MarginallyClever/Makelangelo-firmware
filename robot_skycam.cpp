//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#include "configure.h"
#include "robot_skycam.h"

#if MACHINE_STYLE == SKYCAM

/**
 * Inverse Kinematics turns XY coordinates into lengths of belt from each motor
 * @param axies the cartesian coordinates
 * @param motorStepArray a measure of each belt to that plotter position
 */
void IK(const float *const cartesian, long *motorStepArray) {
  float x = cartesian[0];
  float y = cartesian[1];
  float z = cartesian[2];
  
  float limit_xmax = axies[0].limitMax;
  float limit_xmin = axies[0].limitMin;
  float limit_ymax = axies[1].limitMax;
  float limit_ymin = axies[1].limitMin;
  
  float L,R,U,V,dy,dx,dz;

  dz = z;

  dy = abs(y - limit_ymax)-SKYCAM_COMPENSATION;  dx = abs(x - limit_xmin)-SKYCAM_COMPENSATION;  L = sqrt(dx*dx+dy*dy+dz*dz);  motorStepArray[0] = lround( L / MM_PER_STEP );  // M0 (top left)
  dy = abs(y - limit_ymax)-SKYCAM_COMPENSATION;  dx = abs(x - limit_xmax)-SKYCAM_COMPENSATION;  R = sqrt(dx*dx+dy*dy+dz*dz);  motorStepArray[1] = lround( R / MM_PER_STEP );  // M1 (top right)
  dy = abs(y - limit_ymin)-SKYCAM_COMPENSATION;  dx = abs(x - limit_xmin)-SKYCAM_COMPENSATION;  U = sqrt(dx*dx+dy*dy+dz*dz);  motorStepArray[2] = lround( U / MM_PER_STEP );  // M2 (bottom left)
  dy = abs(y - limit_ymin)-SKYCAM_COMPENSATION;  dx = abs(x - limit_xmax)-SKYCAM_COMPENSATION;  V = sqrt(dx*dx+dy*dy+dz*dz);  motorStepArray[3] = lround( V / MM_PER_STEP );  // M3 (bottom right)
}


/** 
 * Forward Kinematics - turns L1,L2 lengths into XY coordinates
 * @param motorStepArray a measure of each belt to that plotter position
 * @param axies the resulting cartesian coordinate
 * @return 0 if no problem, 1 on failure.
 */
int FK(long *motorStepArray,float *axies) {
  Serial.println(F("UnimplmentedException"));
  return 1;
}



void robot_findHome() {

}


void robot_setup() {
}

#endif

//------------------------------------------------------------------------------
// Makelangelo - a mural drawing robot
// dan@marginallycelver.com 2013-12-26
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/Makelangelo for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == SKYCAM



/**
 * Inverse Kinematics turns XY coordinates into lengths of belt from each motor
 * @param axies the cartesian coordinates
 * @param motorStepArray a measure of each belt to that plotter position
 */
void IK(float *axies, long *motorStepArray) {
  float x = axies[0];
  float y = axies[1];
  float z = axies[2];
  
  float L,R,U,V,dy,dx,dz;

  dz = z;

  dy = abs(y - limit_ymax)-SKYCAM_COMPENSATION;
  dx = abs(x - limit_xmin)-SKYCAM_COMPENSATION;
  L = sqrt(dx*dx+dy*dy+dz*dz);
  motorStepArray[0] = lround( L / THREAD_PER_STEP );  // M0 (top left)
  dy = abs(y - limit_ymax)-SKYCAM_COMPENSATION;
  dx = abs(x - limit_xmax)-SKYCAM_COMPENSATION;
  R = sqrt(dx*dx+dy*dy+dz*dz);
  motorStepArray[1] = lround( R / THREAD_PER_STEP );  // M1 (top right)
  dy = abs(y - limit_ymin)-SKYCAM_COMPENSATION;
  dx = abs(x - limit_xmin)-SKYCAM_COMPENSATION;
  U = sqrt(dx*dx+dy*dy+dz*dz);
  motorStepArray[2] = lround( U / THREAD_PER_STEP );  // M2 (bottom left)
  dy = abs(y - limit_ymin)-SKYCAM_COMPENSATION;
  dx = abs(x - limit_xmax)-SKYCAM_COMPENSATION;
  V = sqrt(dx*dx+dy*dy+dz*dz);
  motorStepArray[3] = lround( V / THREAD_PER_STEP );  // M3 (bottom right)
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


#endif

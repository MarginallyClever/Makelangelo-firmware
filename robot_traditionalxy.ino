//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == TRADITIONALXY


/**
 * Inverse Kinematics turns XY coordinates into step counts from each motor
 * @param x cartesian coordinate
 * @param y cartesian coordinate
 * @param motorStepArray a measure of each belt to that plotter position
 */
void IK(float *axies, long *motorStepArray) {
  float x = axies[0];
  float y = axies[1];
  float z = axies[2];
  
  motorStepArray[0] = lround((x) / THREAD_PER_STEP);
  motorStepArray[1] = lround((y) / THREAD_PER_STEP);

  motorStepArray[NUM_MOTORS] = z;
}


/** 
 * Forward Kinematics - turns step counts into XY coordinates
 * @param motorStepArray a measure of each belt to that plotter position
 * @param axies the resulting cartesian coordinate
 * @return 0 if no problem, 1 on failure.
 */
int FK(long *motorStepArray,float *axies) {
  axies[0] = motorStepArray[0] * THREAD_PER_STEP;
  axies[1] = motorStepArray[1] * THREAD_PER_STEP;
  axies[2] = motorStepArray[NUM_MOTORS];
}



void robot_findHome() {

}


void robot_setup() {
}

#endif

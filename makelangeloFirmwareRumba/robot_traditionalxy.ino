//------------------------------------------------------------------------------
// Makelangelo - a mural drawing robot
// dan@marginallycelver.com 2013-12-26
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/Makelangelo for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == TRADITIONALXY


/**
 * Inverse Kinematics turns XY coordinates into step counts from each motor
 * @param x cartesian coordinate
 * @param y cartesian coordinate
 * @param motorStepArray a measure of each belt to that plotter position
 */
void IK(float x, float y, float z, long *motorStepArray) {
  motorStepArray[0] = lround((x) / THREAD_PER_STEP);
  motorStepArray[1] = lround((y) / THREAD_PER_STEP);

  motorStepArray[NUM_MOTORS] = z;
}


/** 
 * Forward Kinematics - turns step counts into XY coordinates
 * @param motorStepArray a measure of each belt to that plotter position
 * @param x the resulting cartesian coordinate
 * @param y the resulting cartesian coordinate
 */
void FK(long *motorStepArray,float &x,float &y) {
  x = motorStepArray[0] * THREAD_PER_STEP;
  y = motorStepArray[1] * THREAD_PER_STEP;
}


#endif

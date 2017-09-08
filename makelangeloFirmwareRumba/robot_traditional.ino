//------------------------------------------------------------------------------
// Makelangelo - a mural drawing robot
// dan@marginallycelver.com 2013-12-26
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/Makelangelo for more information.
//------------------------------------------------------------------------------

#ifdef TRADITIONALXY


/**
 * Inverse Kinematics turns XY coordinates into step counts from each motor
 * @param x cartesian coordinate
 * @param y cartesian coordinate
 * @param motorStepArray a measure of each belt to that plotter position
 */
void IK(float x, float y, long *motorStepArray) {
  motorStepArray[0] = lround((x) / threadPerStep);
  motorStepArray[1] = lround((y) / threadPerStep);
}


/** 
 * Forward Kinematics - turns step counts into XY coordinates
 * @param motorStepArray a measure of each belt to that plotter position
 * @param x the resulting cartesian coordinate
 * @param y the resulting cartesian coordinate
 */
void FK(long *motorStepArray,float &x,float &y) {
  x = motorStepArray[0] * threadPerStep;
  y = motorStepArray[1] * threadPerStep;
}


#endif

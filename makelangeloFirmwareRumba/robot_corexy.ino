//------------------------------------------------------------------------------
// Makelangelo - a mural drawing robot
// dan@marginallycelver.com 2013-12-26
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/Makelangelo for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == COREXY


/**
 * Inverse Kinematics turns XY coordinates into lengths L1,L2
 * @param x cartesian coordinate
 * @param y cartesian coordinate
 * @param motorStepArray a measure of each belt to that plotter position
 */
void IK(float x, float y, long *motorStepArray) {
  motorStepArray[0] = lround((x+y) / threadPerStep);
  motorStepArray[1] = lround((x-y) / threadPerStep);
}


/** 
 * Forward Kinematics - turns L1,L2 lengths into XY coordinates
 * @param motorStepArray a measure of each belt to that plotter position
 * @param x the resulting cartesian coordinate
 * @param y the resulting cartesian coordinate
 */
void FK(long *motorStepArray,float &x,float &y) {
  float a = motorStepArray[0] * threadPerStep;
  float b = motorStepArray[1] * threadPerStep;

  x = (float)( a + b ) / 2.0;
  y = x - (float)b;
}



#endif

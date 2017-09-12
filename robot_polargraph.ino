//------------------------------------------------------------------------------
// Makelangelo - a mural drawing robot
// dan@marginallycelver.com 2013-12-26
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/Makelangelo for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == POLARGRAPH


/**
 * Inverse Kinematics turns XY coordinates into step counts from each motor
 * @param axies the cartesian coordinate
 * @param motorStepArray a measure of each belt to that plotter position
 */
void IK(float *axies, long *motorStepArray) {
  float dy,dx;
  // find length to M1
  dy = axies[1] - limit_ymax;
  dx = axies[0] - limit_xmin;
  motorStepArray[0] = lround( sqrt(dx*dx+dy*dy) / THREAD_PER_STEP );
  // find length to M2
  dx = limit_xmax - axies[0];
  motorStepArray[1] = lround( sqrt(dx*dx+dy*dy) / THREAD_PER_STEP );

  motorStepArray[NUM_MOTORS] = axies[2];
}


/** 
 * Forward Kinematics - turns step counts into XY coordinates
 * @param motorStepArray a measure of each belt to that plotter position
 * @param axies the resulting cartesian coordinate
 * @return 0 if no problem, 1 on failure.
 */
int FK(long *motorStepArray,float *axies) {
  // use law of cosines: theta = acos((a*a+b*b-c*c)/(2*a*b));
  float a = (float)motorStepArray[0] * THREAD_PER_STEP;
  float b = (limit_xmax-limit_xmin);
  float c = (float)motorStepArray[1] * THREAD_PER_STEP;

  // slow, uses trig
  // we know law of cosines:   cc = aa + bb -2ab * cos( theta )
  // or cc - aa - bb = -2ab * cos( theta )
  // or ( aa + bb - cc ) / ( 2ab ) = cos( theta );
  // or theta = acos((aa+bb-cc)/(2ab));
  //x = cos(theta)*l1 + limit_xmin;
  //y = sin(theta)*l1 + limit_ymax;
  // and we know that cos(acos(i)) = i
  // and we know that sin(acos(i)) = sqrt(1-i*i)
  float theta = ((a*a+b*b-c*c)/(2.0*a*b));
  
  axies[0] = theta * a + limit_xmin;
  axies[1] = limit_ymax - (sqrt( 1.0 - theta * theta ) * a);
  axies[2] = motorStepArray[NUM_MOTORS];
}


#endif

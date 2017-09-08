//------------------------------------------------------------------------------
// Makelangelo - a mural drawing robot
// dan@marginallycelver.com 2013-12-26
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/Makelangelo for more information.
//------------------------------------------------------------------------------

#ifdef POLARGRAPH


/**
 * Inverse Kinematics turns XY coordinates into step counts from each motor
 * @param x cartesian coordinate
 * @param y cartesian coordinate
 * @param motorStepArray a measure of each belt to that plotter position
 */
void IK(float x, float y, long *motorStepArray) {
  float dy,dx;
  // find length to M1
  dy = y - limit_ymax;
  dx = x - limit_xmin;
  motorStepArray[0] = lround( sqrt(dx*dx+dy*dy) / threadPerStep );
  // find length to M2
  dx = limit_xmax - x;
  motorStepArray[1] = lround( sqrt(dx*dx+dy*dy) / threadPerStep );
}


/** 
 * Forward Kinematics - turns step counts into XY coordinates
 * @param motorStepArray a measure of each belt to that plotter position
 * @param x the resulting cartesian coordinate
 * @param y the resulting cartesian coordinate
 */
void FK(long *motorStepArray,float &x,float &y) {
  // use law of cosines: theta = acos((a*a+b*b-c*c)/(2*a*b));
  float a = (float)motorStepArray[0] * threadPerStep;
  float b = (limit_xmax-limit_xmin);
  float c = (float)motorStepArray[1] * threadPerStep;

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
  x = theta * a + limit_xmin;
  y = limit_ymax - (sqrt( 1.0 - theta * theta ) * a);
}


#endif

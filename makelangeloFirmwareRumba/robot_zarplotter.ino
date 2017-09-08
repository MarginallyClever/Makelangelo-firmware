//------------------------------------------------------------------------------
// Makelangelo - a mural drawing robot
// dan@marginallycelver.com 2013-12-26
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/Makelangelo for more information.
//------------------------------------------------------------------------------

#ifdef ZARPLOTTER



/**
 * Inverse Kinematics turns XY coordinates into lengths of belt from each motor
 * @param x cartesian coordinate
 * @param y cartesian coordinate
 * @param motorStepArray a measure of each belt to that plotter position
 */
void IK(float x, float y, long *motorStepArray) {
  float L,R,U,V,dy,dx;
  dy = abs(y - limit_ymax)-ZARPLOTTER_COMPENSATION;  dx = abs(x - limit_xmin)-ZARPLOTTER_COMPENSATION;  L = sqrt(dx*dx+dy*dy);  motorStepArray[0] = lround( L / threadPerStep );  // M0 (top left)
  dy = abs(y - limit_ymax)-ZARPLOTTER_COMPENSATION;  dx = abs(x - limit_xmax)-ZARPLOTTER_COMPENSATION;  R = sqrt(dx*dx+dy*dy);  motorStepArray[1] = lround( R / threadPerStep );  // M1 (top right)
  dy = abs(y - limit_ymin)-ZARPLOTTER_COMPENSATION;  dx = abs(x - limit_xmin)-ZARPLOTTER_COMPENSATION;  U = sqrt(dx*dx+dy*dy);  motorStepArray[2] = lround( U / threadPerStep );  // M2 (bottom left)
  dy = abs(y - limit_ymin)-ZARPLOTTER_COMPENSATION;  dx = abs(x - limit_xmax)-ZARPLOTTER_COMPENSATION;  V = sqrt(dx*dx+dy*dy);  motorStepArray[3] = lround( V / threadPerStep );  // M3 (bottom right)
/*
  Serial.print(x);  Serial.print(' ');
  Serial.print(y);  Serial.print(' ');
  Serial.print(L);  Serial.print(' ');
  Serial.print(R);  Serial.print(' ');
  Serial.print(U);  Serial.print(' ');
  Serial.print(V);  Serial.print('\n');
*/
}


/** 
 * Forward Kinematics - turns L1,L2 lengths into XY coordinates
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
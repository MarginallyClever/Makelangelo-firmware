//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#include "configure.h"

#if MACHINE_STYLE == ZARPLOTTER

/**
 * Inverse Kinematics turns XY coordinates into lengths of belt from each motor
 * @param axies the cartesian coordinates
 * @param motorStepArray a measure of each belt to that plotter position
 */
void IK(const float *const cartesian, long *motorStepArray) {
  float left   = axies[0].limitMin+ZARPLOTTER_COMPENSATION;
  float right  = axies[0].limitMax-ZARPLOTTER_COMPENSATION;
  float top    = axies[1].limitMax-ZARPLOTTER_COMPENSATION;
  float bottom = axies[1].limitMin+ZARPLOTTER_COMPENSATION;
  float x = cartesian[0];
  float y = cartesian[1];
  
  float L,R,U,V,dy,dx;

  // clockwise from top left.
  dx = x-left ;  dy = y-top   ;  motorStepArray[0] = lroundf( (sqrt(sq(dx)+sq(dy))) / MM_PER_STEP );  // M0 (top left)
  dx = x-right;  dy = y-top   ;  motorStepArray[1] = lroundf( (sqrt(sq(dx)+sq(dy))) / MM_PER_STEP );  // M1 (top right)
  dx = x-right;  dy = y-bottom;  motorStepArray[2] = lroundf( (sqrt(sq(dx)+sq(dy))) / MM_PER_STEP );  // M2 (bottom right)
  dx = x-left ;  dy = y-bottom;  motorStepArray[3] = lroundf( (sqrt(sq(dx)+sq(dy))) / MM_PER_STEP );  // M3 (bottom left)
/*
  Serial.print(x);
  Serial.print('\t');
  Serial.print(y);
  Serial.print('\t');
  Serial.print(motorStepArray[0]);
  Serial.print('\t');
  Serial.print(motorStepArray[1]);
  Serial.print('\t');
  Serial.print(motorStepArray[2]);
  Serial.print('\t');
  Serial.print(motorStepArray[3]);
  Serial.print('\n');
  */
  motorStepArray[NUM_MOTORS] = cartesian[2];
/*
  Serial.print(cartesian[0]);  Serial.print('\t');
  Serial.print(cartesian[1]);  Serial.print('\t');
  Serial.print(L);  Serial.print('\t');
  Serial.print(R);  Serial.print('\t');
  Serial.print(U);  Serial.print('\t');
  Serial.print(V);  Serial.println();
*/
}


/** 
 * Forward Kinematics - turns L1,L2 lengths into XY coordinates
 * @param motorStepArray a measure of each belt to that plotter position
 * @param axies the resulting cartesian coordinate
 * @return 0 if no problem, 1 on failure.
 */
int FK(long *motorStepArray, float *cartesian) {
  float limit_xmin = axies[0].limitMin;
  float limit_xmax = axies[0].limitMax;
  float limit_ymax = axies[1].limitMax;
  
  // use law of cosines: theta = acos((a*a+b*b-c*c)/(2*a*b));
  float a = (float)motorStepArray[0] * MM_PER_STEP;
  float b = (limit_xmax-limit_xmin);
  float c = (float)motorStepArray[1] * MM_PER_STEP;

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
  
  cartesian[0] = theta * a + limit_xmin;
  cartesian[1] = limit_ymax - (sqrt( 1.0 - theta * theta ) * a);
  cartesian[2] = motorStepArray[NUM_MOTORS];

  return 0;
}



void robot_findHome() {

}


void robot_setup() {
}

#endif

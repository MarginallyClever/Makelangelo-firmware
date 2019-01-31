//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#include "configure.h"
#include "robot_zarplotter.h"

#if MACHINE_STYLE == ZARPLOTTER

/**
 * Inverse Kinematics turns XY coordinates into lengths of belt from each motor
 * @param axies the cartesian coordinates
 * @param motorStepArray a measure of each belt to that plotter position
 */
void IK(float *cartesian, long *motorStepArray) {
  float limit_xmin = axies[0].limitMin;
  float limit_xmax = axies[0].limitMax;
  float limit_ymin = axies[1].limitMin;
  float limit_ymax = axies[1].limitMax;
  
  float x = cartesian[0];
  float y = cartesian[1];
  float z = cartesian[2];
  
  float L,R,U,V,dy,dx;
  dy = abs(y - limit_ymax)-ZARPLOTTER_COMPENSATION;  dx = abs(x - limit_xmin)-ZARPLOTTER_COMPENSATION;  L = sqrt(dx*dx+dy*dy);  motorStepArray[0] = lround( L / THREAD_PER_STEP );  // M0 (top left)
  dy = abs(y - limit_ymax)-ZARPLOTTER_COMPENSATION;  dx = abs(x - limit_xmax)-ZARPLOTTER_COMPENSATION;  R = sqrt(dx*dx+dy*dy);  motorStepArray[1] = lround( R / THREAD_PER_STEP );  // M1 (top right)
  dy = abs(y - limit_ymin)-ZARPLOTTER_COMPENSATION;  dx = abs(x - limit_xmin)-ZARPLOTTER_COMPENSATION;  U = sqrt(dx*dx+dy*dy);  motorStepArray[2] = lround( U / THREAD_PER_STEP );  // M2 (bottom left)
  dy = abs(y - limit_ymin)-ZARPLOTTER_COMPENSATION;  dx = abs(x - limit_xmax)-ZARPLOTTER_COMPENSATION;  V = sqrt(dx*dx+dy*dy);  motorStepArray[3] = lround( V / THREAD_PER_STEP );  // M3 (bottom right)
/*
  Serial.print(x);  Serial.print(' ');
  Serial.print(y);  Serial.print(' ');
  Serial.print(L);  Serial.print(' ');
  Serial.print(R);  Serial.print(' ');
  Serial.print(U);  Serial.print(' ');
  Serial.print(V);  Serial.print('\n');
*/

  motorStepArray[NUM_MOTORS] = z;
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
  
  cartesian[0] = theta * a + limit_xmin;
  cartesian[1] = limit_ymax - (sqrt( 1.0 - theta * theta ) * a);
  cartesian[2] = motorStepArray[NUM_MOTORS];
}



void robot_findHome() {

}


void robot_setup() {
}

#endif

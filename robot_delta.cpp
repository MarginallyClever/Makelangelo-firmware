//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#include "configure.h"
#include "robot_delta.h"

#if MACHINE_STYLE == DELTA

#define SQRT3   (sqrt(3.0))
#define SIN120  (SQRT3/2.0)
#define COS120  (-0.5)
#define TAN60   (SQRT3)
#define SIN30   (0.5)
#define TAN30   (1.0/SQRT3)


/**
 * Inverse Kinematics turns XY coordinates into step counts from each motor
 * @param axies the cartesian coordinate
 * @param motorStepArray a measure of each belt to that plotter position
 */
void IK(const float *const axies, long *motorStepArray) {
  float x = axies[0];
  float y = axies[1];
  float z = axies[2];

  motorStepArray[0]=0;
  motorStepArray[1]=0;
  motorStepArray[2]=0;
  
  int state = delta_calcAngleYZ(x, y, z, motorStepArray[0]);
  if(state == 0) {
    state = delta_calcAngleYZ(x*COS120 + y*SIN120, y*COS120-x*SIN120, z, motorStepArray[1]);  // rotate coords to +120 deg
  }
  if(state == 0) {
    state = delta_calcAngleYZ(x*COS120 - y*SIN120, y*COS120+x*SIN120, z, motorStepArray[2]);  // rotate coords to -120 deg
  }

#if NUM_AXIES>3
  // if there is a fourth axis rotation on the head of the delta, transfer it here.
  motorStepArray[3] = axies[3] * MICROSTEP_PER_DEGREE;
#endif
  
  //Serial.print("IK ");
  //Serial.print('\t');  Serial.print(axies[0]);
  //Serial.print('\t');  Serial.print(axies[1]);
  //Serial.print('\t');  Serial.print(axies[2]);
  //Serial.print('\t');  Serial.print(motorStepArray[0]);
  //Serial.print('\t');  Serial.print(motorStepArray[1]);
  //Serial.print('\t');  Serial.print(motorStepArray[2]);
  //Serial.print('\n');
}


/** 
 * Forward Kinematics - turns step counts into XY coordinates
 * @param motorStepArray a measure of each belt to that plotter position
 * @param axies the resulting cartesian coordinate
 * @return 0 if no problem, 1 on failure.
 */
int FK(long *motorStepArray,float *axies) {
  float t = (CENTER_TO_SHOULDER-EFFECTOR_TO_WRIST)*TAN30/2.0;
  float theta1 = radians((float)motorStepArray[0]/MICROSTEP_PER_DEGREE);
  float theta2 = radians((float)motorStepArray[1]/MICROSTEP_PER_DEGREE);
  float theta3 = radians((float)motorStepArray[2]/MICROSTEP_PER_DEGREE);

  float y1 = -(t + SHOULDER_TO_ELBOW*cos(theta1));
  float z1 = -SHOULDER_TO_ELBOW*sin(theta1);

  float y2 = (t + SHOULDER_TO_ELBOW*cos(theta2))*SIN30;
  float x2 = y2*TAN60;
  float z2 = -SHOULDER_TO_ELBOW*sin(theta2);

  float y3 = (t + SHOULDER_TO_ELBOW*cos(theta3))*SIN30;
  float x3 = -y3*TAN60;
  float z3 = -SHOULDER_TO_ELBOW*sin(theta3);

  float dnm = (y2-y1)*x3-(y3-y1)*x2;

  float w1 = y1*y1 + z1*z1;
  float w2 = x2*x2 + y2*y2 + z2*z2;
  float w3 = x3*x3 + y3*y3 + z3*z3;

  // x = (a1*z + b1)/dnm
  float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
  float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;

  // y = (a2*z + b2)/dnm;
  float a2 = -(z2-z1)*x3+(z3-z1)*x2;
  float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;

  // a*z^2 + b*z + c = 0
  float a = a1*a1 + a2*a2 + dnm*dnm;
  float b = 2.0*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
  float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - ELBOW_TO_WRIST*ELBOW_TO_WRIST);

  // discriminant
  float d = b*b - 4.0*a*c;
  if (d < 0.0) return 1; // no intersection.
  
  float z0 = -0.5*(b+sqrt(d))/a;
  float x0 = (a1*z0 + b1)/dnm;
  float y0 = (a2*z0 + b2)/dnm;

  axies[0]=x0;
  axies[1]=y0;
  axies[2]=z0;
  return 0;
}


/** 
 * inverse kinematics helper function.  calculates angle theta1 (for YZ-pane) 
 * @param x0 
 * @param y0 
 * @param z0 
 * @param theta to be filled with newly calculated angle
 * @return 1 on failure, 0 on success
 */
int delta_calcAngleYZ(float x0, float y0, float z0, long &theta) {
  float y1 = -0.5 * TAN30 * CENTER_TO_SHOULDER;  // f/2 * tg 30
  
  z0-= CENTER_TO_FLOOR;
   
  y0 -= 0.5 * TAN30 * EFFECTOR_TO_WRIST;  // shift center to edge
  // z = a + b*y
  float a = (x0*x0 + y0*y0 + z0*z0 +SHOULDER_TO_ELBOW*SHOULDER_TO_ELBOW - ELBOW_TO_WRIST*ELBOW_TO_WRIST - y1*y1)/(2.0*z0);
  float b = (y1-y0)/z0;

  //Serial.print("a=");  Serial.println(a);
  //Serial.print("b=");  Serial.println(b);

  // discriminant
  float d = -(a+b*y1)*(a+b*y1)+SHOULDER_TO_ELBOW*(b*b*SHOULDER_TO_ELBOW+SHOULDER_TO_ELBOW); 
  //Serial.print("d=");  Serial.println(d);
  if (d < 0) return 1; // non-existing povar.  return error, theta


  float yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer povar
  float zj = a + b*yj;
  theta = atan(-zj/(y1 - yj)) * 180.0/PI + ((yj>y1)?180.0:0.0);
  theta *= MICROSTEP_PER_DEGREE;
  
  //Serial.print("yj=");  Serial.println(yj);
  //Serial.print("zj=");  Serial.println(zj);
  //Serial.print("theta=");  Serial.println(theta);

  return 0;  // return error, theta
}


void robot_findHome() {
  char i;

  motor_disengage();

  // back up until the arms hit the limit switches
  float horizontal = DEGREES_ABOVE_HORIZONTAL;
  long j, steps_to_zero = horizontal * MICROSTEP_PER_DEGREE;

  for(i=0;i<NUM_MOTORS;++i) {
    // enable one motor at a time
    digitalWrite(motors[i].enable_pin,LOW);
    digitalWrite(motors[i].dir_pin, DELTA_HOME_DIRECTION);
    
    // drive until you hit the switch
    while(digitalRead(motors[i].limit_switch_pin) == HIGH) {
      digitalWrite(motors[i].step_pin, HIGH);
      digitalWrite(motors[i].step_pin, LOW);
      pause(step_delay);
    }
    
    // move to home position
    digitalWrite(motors[i].dir_pin, DELTA_HOME_DIRECTION==LOW?HIGH:LOW);
    for(j=0;j<steps_to_zero;++j) {
      Serial.println(i,DEC);
      digitalWrite(motors[i].step_pin, HIGH);
      digitalWrite(motors[i].step_pin, LOW);
      pause(step_delay);
    }
  }

  Serial.println("Done.");
  float aa = CENTER_TO_SHOULDER + SHOULDER_TO_ELBOW - EFFECTOR_TO_WRIST;
  float cc = ELBOW_TO_WRIST;
  float bb = sqrt(cc*cc - aa*aa);
  //Serial.print("aa=");  Serial.println(aa);
  //Serial.print("bb=");  Serial.println(bb);
  //Serial.print("cc=");  Serial.println(cc);
  axies[0].pos=0;
  axies[1].pos=0;
  axies[2].pos = CENTER_TO_FLOOR - bb;
}


void robot_setup() {
}



#endif

//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == STEWART


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
void IK(float *axies, long *motorStepArray) {
  float x = axies[0];
  float y = axies[1];
  float z = axies[2];
  float u = axies[3];
  float v = axies[4];
  float w = axies[5];

}


/** 
 * Forward Kinematics - turns step counts into XY coordinates
 * @param motorStepArray a measure of each belt to that plotter position
 * @param axies the resulting cartesian coordinate
 * @return 0 if no problem, 1 on failure.
 */
int FK(long *motorStepArray,float *axies) {
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
int stewart_calcAngleYZ(float x0, float y0, float z0, long &theta) {
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
  motor_enable();
  
  char i;
  // until all switches are hit
  while(stewart_read_switches()<6) {
#if VERBOSE > 0
  Serial.println(stewart_read_switches(),DEC);
#endif
    // for each stepper,
    for(i=0;i<6;++i) {
      // if this switch hasn't been hit yet
      if(robot.arms[i].limit_switch_state == HIGH) {
        // move "down"
        digitalWrite(motors[i].dirPin,LOW);
        digitalWrite(motors[i].stepPin,HIGH);
        digitalWrite(motors[i].stepPin,LOW);
      }
    }
    pause(250);
  }

  // The arms are 19.69 degrees from straight down when they hit the switcrobot.
  // @TODO: This could be better customized in firmware.
  stewart_loadHomeAngles();
  
  Serial.println(F("Homing..."));
#if VERBOSE > 0
  Serial.print("steps=");
  Serial.println(steps_to_zero);
#endif
  char keepGoing;
  do {
    for(i=0;i<NUM_AXIES;++i) {
      keepGoing=0;
      if(robot.steps_to_zero[i]>0) {
        --robot.steps_to_zero[i];
        digitalWrite(motors[i].dirPin,HIGH);
        digitalWrite(motors[i].stepPin,HIGH);
        digitalWrite(motors[i].stepPin,LOW);
        keepGoing=1;
      }
    }
    pause(250);
  } while(keepGoing>0);
    
  // recalculate XYZ positions
}


/**
 * read the limit switch states
 * @return 1 if a switch is being hit
 */
char stewart_read_switches() {
  char i, hit=0;
  int state;
  
  for(i=0;i<NUM_MOTORS;++i) {
    state=digitalRead(motors[i].limit_switch_pin);
#if DEBUG_SWITCHES > 0
    Serial.print(state);
    Serial.print('\t');
#endif
    if(motors[i].limit_switch_state != state) {
      motors[i].limit_switch_state = state;
#if DEBUG_SWITCHES > 0
      Serial.print(F("Switch "));
      Serial.println(i,DEC);
#endif
    }
    if(state == LOW) ++hit;
  }
#if DEBUG_SWITCHES > 0
  Serial.print('\n');
#endif
  return hit;
}


#endif

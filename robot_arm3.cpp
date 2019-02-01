//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#include "configure.h"
#include "robot_arm3.h"

#if MACHINE_STYLE == ARM3

/**
 * Inverse Kinematics turns XY coordinates into step counts from each motor
 * @param x cartesian coordinate
 * @param y cartesian coordinate
 * @param motorStepArray a measure of each belt to that plotter position
 */
void IK(const float *const axies, long *motorStepArray) {
  float x = axies[0];
  float y = axies[1];
  float z = axies[2];

  // if we know the position of the wrist relative to the shoulder
  // we can use intersection of circles to find the elbow.
  // once we know the elbow position we can find the angle of each joint.
  // each angle can be converted to motor steps.
    
  // use intersection of circles to find two possible elbow points.
  // the two circles are the bicep (shoulder-elbow) and the forearm (elbow-wrist)
  // the distance between circle centers is d  
  Vector3 arm_plane(x,y,0);
  arm_plane.Normalize();

  // the finger (attachment point for the tool) is a short distance in "front" of the wrist joint
  Vector3 wrist(x,y,z);
  wrist -= arm_plane * WRIST_TO_FINGER;

  Vector3 shoulder = arm_plane;
  shoulder *= BASE_TO_SHOULDER_X;
  shoulder.z = BASE_TO_SHOULDER_Z;
  
  Vector3 es = wrist - shoulder;
  
  float d = es.Length();
  
  //a = (r0r0 - r1r1 + d*d ) / (2 d) 
  float r1=ELBOW_TO_WRIST;  // circle 1 centers on wrist
  float r0=SHOULDER_TO_ELBOW;  // circle 0 centers on shoulder
  if( d > ELBOW_TO_WRIST + SHOULDER_TO_ELBOW ) {
    // The points are impossibly far apart, no solution can be found.
    return;
  }
    
  float a = ( r0 * r0 - r1 * r1 + d*d ) / ( 2.0*d );
  // find the midpoint
  Vector3 mid = es * ( a / d ) + shoulder;
  // with a and r0 we can find h, the distance from midpoint to the intersections.
  float h=sqrt(r0*r0-a*a);
  // the distance h on a line orthogonal to n and plane_normal gives us the two intersections.
  Vector3 n(-arm_plane.y,arm_plane.x,0);
  Vector3 r = es ^ n;
  r.Normalize();
  Vector3 elbow = mid - r * h;
  //Vector3 elbow = mid + r * h;
    
  // find the shoulder angle using atan3(elbow-shoulder)
  Vector3 temp = elbow - shoulder;
  temp.Normalize();
  float ax=temp | arm_plane;
  float ay=temp.z;
  motorStepArray[1] = atan2(ay,ax);

  // find the elbow angle
  temp = elbow - wrist;
  temp.Normalize();
  float bx = temp | arm_plane;
  float by = temp.z;
  motorStepArray[0] = -atan2(by,bx);
  
  // the easiest part
  motorStepArray[2] = -atan2(y,x);

  // angles are now in radians
  
#if VERBOSE > 2
  Serial.print(x);
  Serial.print("\t");
  Serial.print(y);
  Serial.print("\t");
  Serial.print(z);
  Serial.print("\t=\t");
  Serial.print(angle_0*RAD2DEG);
  Serial.print("\t");
  Serial.print(angle_1*RAD2DEG);
  Serial.print("\t");
  Serial.print(angle_2*RAD2DEG);
  Serial.print("\n");
#endif

  motorStepArray[0] *= STEPS_PER_TURN*GEAR_RATIO/(PI*2.0f);
  motorStepArray[1] *= STEPS_PER_TURN*GEAR_RATIO/(PI*2.0f);
  motorStepArray[2] *= STEPS_PER_TURN*GEAR_RATIO/(PI*2.0f);
}


/** 
 * Forward Kinematics - turns step counts into XY coordinates
 * @param motorStepArray a measure of each belt to that plotter position
 * @param axies the resulting cartesian coordinate
 * @return 0 if no problem, 1 on failure.
 */
int FK(long *motorStepArray,float *axies) {
}



void robot_findHome() {
  // AXIS 1
  
  // hit switch
  digitalWrite(motors[1].dir_pin,LOW);
  while(digitalRead(motors[1].limit_switch_pin)==HIGH) {
    motor_onestep(1);
    pause(500);
  }
  // Back off switch
  digitalWrite(motors[1].dir_pin,HIGH);
  while(digitalRead(motors[1].limit_switch_pin)==LOW) {
    motor_onestep(1);
    pause(500);
  }
#if VERBOSE > 1
  Serial.println(F("Found 1"));
#endif

  // AXIS 2
  // hit switch
  digitalWrite(motors[0].dir_pin,LOW);
  while(digitalRead(motors[0].limit_switch_pin)==HIGH) {
    motor_onestep(0);
    pause(500);
  }
  // Back off switch
  digitalWrite(motors[0].dir_pin,HIGH);
  while(digitalRead(motors[0].limit_switch_pin)==LOW) {
    motor_onestep(0);
    pause(500);
  }
#if VERBOSE > 1
  Serial.println(F("Found 0"));
#endif

  // hit switch
  digitalWrite(motors[2].dir_pin,LOW);
  while(digitalRead(motors[2].limit_switch_pin)==HIGH) {
    motor_onestep(2);
    pause(500);
  }
  // Back off switch
  digitalWrite(motors[2].dir_pin,HIGH);
  while(digitalRead(motors[2].limit_switch_pin)==LOW) {
    motor_onestep(2);
    pause(500);
  }
#if VERBOSE > 1
  Serial.println(F("Found 2"));
#endif

  float homePos[3] = {HOME_X,HOME_Y,HOME_Z};  // set staring position
  teleport(homePos);
  
  Serial.println(F("Found home."));
}


void robot_setup() {
}


#endif  // ARM3

  //------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == ARM6

#include "Vector3.h"

/**
   Inverse Kinematics turns XY coordinates into step counts from each motor
   This code is a duplicate of https://github.com/MarginallyClever/Robot-Overlord-App/blob/master/src/main/java/com/marginallyclever/robotOverlord/sixiRobot/java inverseKinematics()
   @param axies the cartesian coordinate
   @param motorStepArray a measure of each belt to that plotter position
*/
void IK(float *axies, long *motorStepArray) {
  float x = axies[0];
  float y = axies[1];
  float z = axies[2];
  float u = axies[3];
  float v = axies[4];
  float w = axies[5];

  float ee, n, xx, yy, angle0,angle1,angle2,angle3,angle4,angle5;

  Vector3 fingerPosition(x,y,z);
  Vector3 fingerForward(0,0,1);
  Vector3 fingerRight(1,0,0);

  //Vector3 up(forward ^ right);
  Vector3 axis_of(0,0,1);
  Vector3 axis_or(1,0,0);
  Vector3 axis_ou(0,1,0);
  
  fingerForward.rotate(axis_of,radians(u));
  fingerForward.rotate(axis_or,radians(v));
  fingerForward.rotate(axis_ou,radians(w));

  fingerRight.rotate(axis_of,radians(u));
  fingerRight.rotate(axis_or,radians(v));
  fingerRight.rotate(axis_ou,radians(w));

  // rotation at finger, bend at wrist, rotation between wrist and elbow, then bends down to base.

  // get the finger position
  Vector3 fingerPlaneZ(fingerForward);
  Vector3 fingerPlaneX(fingerRight);
  Vector3 fingerPlaneY(fingerPlaneZ ^ fingerPlaneX);

  // find the wrist position
  Vector3 wristToFinger(fingerPlaneZ * OFFSET_5Z);  
  Vector3 wristPosition(fingerPosition - wristToFinger);
  
  // I also need part of the base/shoulder to work from the other end of the problem.
  // if I have the wrist and the shoulder then I can make reasonable guesses about the elbow.
  Vector3 shoulderPosition(OFFSET_1X,OFFSET_1Y,OFFSET_1Z);
  
  // Find the facingDirection and planeNormal vectors.
  if(abs(wristPosition.x)<EPSILON && abs(wristPosition.y)<EPSILON) {
    // Wrist is directly above shoulder, makes calculations hard.
    // TODO figure this out.  Use previous state to guess elbow?
    return false;
  }
  Vector3 shoulderPlaneX(wristPosition.x,wristPosition.y,0);
  shoulderPlaneX.normalize();
  Vector3 shoulderPlaneZ(0,0,1);
  Vector3 shoulderPlaneY(shoulderPlaneX ^ shoulderPlaneZ);
  shoulderPlaneY.normalize();

  // Find elbow by using intersection of circles (http://mathworld.wolfram.com/Circle-CircleIntersection.html)
  // x = (dd-rr+RR) / (2d)
  Vector3 shoulderToWrist(wristPosition - shoulderPosition);
  float d = shoulderToWrist.length();
  float R = abs(SHOULDER_TO_ELBOW);
  float r = abs(ELBOW_TO_WRIST);
  if( d > R+r ) {
    // impossibly far away
    return false;
  }
  float x = (d*d - r*r + R*R ) / (2*d);
  if( x > R ) {
    // would cause sqrt(-something)
    return false;
  }
  shoulderToWrist.normalize();
  Vector3 elbowPosition(shoulderToWrist*x + shoulderPosition);
  // v1 is now at the intersection point between ik_wrist and ik_boom
  float a = (float)( sqrt( R*R - x*x ) );
  Vector3 v1(shoulderPlaneY ^ shoulderToWrist);
  elbowPosition.add(v1 * a);

  // All the joint locations are now known.
  // Now I have to build some matrices to find the correct angles because the sixi has those L shaped bones.
  Vector3 elbowToWrist(wristPosition - elbowPosition);
  elbowToWrist.normalize();
  v1 = (elbowToWrist ^ shoulderPlaneY);
  Vector3 v2(v1 ^ shoulderPlaneY);
  v2.normalize();  // normalized version of elbowToWrist 
  
  Vector3 nvx();
  Vector3 nvy();

  nvx = (v1 * cos(radians(ADJUST_WRIST_ELBOW_ANGLE)));
  nvy = (v2 * sin(radians(ADJUST_WRIST_ELBOW_ANGLE)));
  Vector3 elbowPlaneX(nvx + nvy);
  elbowPlaneX.normalize();
  Vector3 elbowPlaneZ(shoulderPlaneY ^ elbowPlaneX);


  Vector3 shoulderToElbow(elbowPosition - shoulderPosition);
  shoulderToElbow.normalize();
  
  v1 = shoulderToElbow;
  v2 = shoulderPlaneY ^ v1;
  nvx = (v1 * cos(radians(ADJUST_SHOULDER_ELBOW_ANGLE)));
  nvy = (v2 * sin(radians(ADJUST_SHOULDER_ELBOW_ANGLE)));
  Vector3 bicepPlaneZ(nvx + nvy);
  bicepPlaneZ.normalize();
  Vector3 bicepPlaneX(bicepPlaneZ ^ shoulderPlaneY);
  
  // ulna matrix
  Vector3 ulnaPlaneZ(elbowPlaneZ);
  
  // I have wristToFinger.  I need wristToFinger projected on the plane elbow-space XY to calculate the angle. 
  float tf = elbowPlaneZ.dot(wristToFinger);
  // v0 and keyframe.fingerForward are normal length.  if they dot to nearly 1, they are colinear.
  // if they are colinear then I have no reference to calculate the angle of the ulna rotation.
  if(tf>=1-EPSILON) {
    return false;
  }

  Vector3 projectionAmount(elbowPlaneZ);
  projectionAmount.scale(tf);
  Vector3 ulnaPlaneX(wristToFinger - projectionAmount);
  ulnaPlaneX.normalize();
  Vector3 ulnaPlaneY(ulnaPlaneX ^ ulnaPlaneZ);
  ulnaPlaneY.normalize();

  // TODO wrist may be bending backward.  As it passes the middle a singularity can occur.
  // Compare projected vector to previous frame's projected vector. if the direction is reversed, flip it. 

  // wrist matrix
  Vector3 wristPlaneZ(wristToFinger);
  wristPlaneZ.normalize();
  Vector3 wristPlaneY(ulnaPlaneY);
  Vector3 wristPlaneX(wristPlaneY ^ wristPlaneZ);
  wristPlaneX.normalize();
  
  // find the angles

  // shoulder
  ee = atan2(shoulderPlaneX.y, shoulderPlaneX.x);
  ee = MathHelper.capRotationRadians(ee);
  angle0 = capRotationDegrees(toDegrees(ee));

  // bicep
  xx = (float)shoulderToElbow.z;
  yy = shoulderPlaneX.dot(shoulderToElbow);
  ee = atan2(yy, xx);
  angle1 = capRotationDegrees(toDegrees(ee)+90-ADJUST_SHOULDER_ELBOW_ANGLE);
  
  // elbow
  xx = (float)shoulderToElbow.dot(elbowToWrist);
  v1.cross(shoulderPlaneY,shoulderToElbow);
  yy = elbowToWrist.dot(v1);
  ee = atan2(yy, xx);
  angle2 = capRotationDegrees(toDegrees(ee)+180-ADJUST_SHOULDER_ELBOW_ANGLE-ADJUST_WRIST_ELBOW_ANGLE);
  
  // ulna rotation
  xx = shoulderPlaneY.dot(ulnaPlaneX);  // shoulderPlaneY is the same as elbowPlaneY
  yy = elbowPlaneX   .dot(ulnaPlaneX);
  ee = atan2(yy, xx);
  angle3 = capRotationDegrees(toDegrees(ee)+90);
  
  // wrist
  xx = ulnaPlaneX.dot(wristToFinger);
  yy = ulnaPlaneZ.dot(wristToFinger);
  ee = atan2(yy, xx);
  angle4 = capRotationDegrees(toDegrees(ee)-90);
  
  // hand   
  xx = wristPlaneY.dot(keyframe.fingerRight);
  yy = wristPlaneX.dot(keyframe.fingerRight);
  ee = atan2(yy, xx);
  angle5 = capRotationDegrees(toDegrees(ee)-90);
  
  motorStepArray[0] = angle0 * MOTOR_0_STEPS_PER_TURN / 360.0;
  motorStepArray[1] = angle1 * MOTOR_1_STEPS_PER_TURN / 360.0;
  motorStepArray[2] = angle2 * MOTOR_2_STEPS_PER_TURN / 360.0;
  motorStepArray[3] = angle3 * MOTOR_3_STEPS_PER_TURN / 360.0;
  motorStepArray[4] = angle4 * MOTOR_4_STEPS_PER_TURN / 360.0;
  motorStepArray[5] = angle5 * MOTOR_5_STEPS_PER_TURN / 360.0;
  motorStepArray[NUM_MOTORS] = axies[6];
}

#define CAP_LIMIT 360
float capRotationDegrees(double arg0) {
  while(arg0<0        ) arg0 += CAP_LIMIT;
  while(arg0>CAP_LIMIT) arg0 -= CAP_LIMIT;
  return arg0;
}

/**
   Forward Kinematics - turns step counts into XY coordinates.  
   This code is a duplicate of https://github.com/MarginallyClever/Robot-Overlord-App/blob/master/src/main/java/com/marginallyclever/robotOverlord/sixiRobot/java forwardKinematics()
   @param motorStepArray a measure of each belt to that plotter position
   @param axies the resulting cartesian coordinate
   @return 0 if no problem, 1 on failure.
*/
int FK(long *motorStepArray, float *axies) {
  float angle0rad = motorStepArray[0] * PI*2.0 / MOTOR_0_STEPS_PER_TURN;
  float angle1rad = motorStepArray[1] * PI*2.0 / MOTOR_1_STEPS_PER_TURN;
  float angle2rad = motorStepArray[2] * PI*2.0 / MOTOR_2_STEPS_PER_TURN;
  float angle3rad = motorStepArray[3] * PI*2.0 / MOTOR_3_STEPS_PER_TURN;
  float angle4rad = motorStepArray[4] * PI*2.0 / MOTOR_4_STEPS_PER_TURN;
  float angle5rad = motorStepArray[5] * PI*2.0 / MOTOR_5_STEPS_PER_TURN;

  Vector3 nvx, nvz, vx, vz;

  Vector3 shoulderPosition(OFFSET_1X,OFFSET_1Y,OFFSET_1Z);
  Vector3 shoulderPlaneZ(0,0,1);
  Vector3 shoulderPlaneX(cos(angle0rad),sin(angle0rad),0);
  Vector3 shoulderPlaneY(shoulderPlaneX ^ shoulderPlaneZ);
  shoulderPlaneY.Normalize();

  // get rotation at bicep
  nvx = (shoulderPlaneX*cos(angle1rad));
  nvz = (shoulderPlaneZ*sin(angle1rad));

  Vector3 bicepPlaneY(shoulderPlaneY);
  Vector3 bicepPlaneZ(nvx+nvz);
  bicepPlaneZ.Normalize();
  Vector3 bicepPlaneX(bicepPlaneZ ^ bicepPlaneY);
  bicepPlaneX.Normalize();

  // shoulder to elbow
  vx = (bicepPlaneX*OFFSET_2Y);
  vz = (bicepPlaneZ*OFFSET_2Z);
  Vector3 shoulderToElbow(vx+vz);
  Vector3 elbowPosition(shoulderPosition+shoulderToElbow);

  // get the matrix at the elbow
  nvx = (bicepPlaneZ*cos(angle2rad));
  nvz = (bicepPlaneX*sin(angle2rad));

  Vector3 elbowPlaneY(shoulderPlaneY);
  Vector3 elbowPlaneZ(nvx+nvz);
  elbowPlaneZ.Normalize();
  Vector3 elbowPlaneX(elbowPlaneZ ^ elbowPlaneY);
  elbowPlaneX.Normalize();

  // get elbow to ulna
  vx = (elbowPlaneX*OFFSET_3Y);
  vz = (elbowPlaneZ*OFFSET_3Z);
  Vector3 elbowToUlna(vx+vz);
  Vector3 ulnaPosition(elbowPosition+elbowToUlna);

  // get matrix of ulna rotation
  Vector3 ulnaPlaneZ(nvx+nvz);
  ulnaPlaneZ.Normalize();
  vx = (elbowPlaneX*cos(angle3rad));
  vz = (elbowPlaneY*sin(angle3rad));
  Vector3 ulnaPlaneX(vx+vz);
  ulnaPlaneX.Normalize();
  Vector3 ulnaPlaneY(ulnaPlaneX ^ ulnaPlaneZ);
  ulnaPlaneY.Normalize();

  Vector3 ulnaToWrist(ulnaPlaneZ*OFFSET_4Z);
  Vector3 wristPosition(ulnaPosition+ulnaToWrist);
  
  // wrist to finger
  vx = (ulnaPlaneZ*cos(angle4rad));
  vz = (ulnaPlaneX*sin(angle4rad));
  Vector3 wristToFingerNormalized(vx+vz);
  wristToFingerNormalized.Normalize();
  Vector3 wristToFinger(wristToFingerNormalized*OFFSET_5Z);
  
  Vector3 wristPlaneY(ulnaPlaneY);
  Vector3 wristPlaneZ(wristToFingerNormalized);
  Vector3 wristPlaneX(wristPlaneY ^ wristPlaneZ);
  wristPlaneX.Normalize();
  
  // finger rotation
  Vector3 fingerPlaneZ(wristPlaneZ);
  vx = (wristPlaneX*cos(angle5rad));
  vz = (wristPlaneY*sin(angle5rad));
  Vector3 fingerPlaneX(vx+vz);
  fingerPlaneX.Normalize();
  Vector3 fingerPlaneY(fingerPlaneZ ^ fingerPlaneX);
  Vector3 fingerPosition(wristPosition+wristToFinger);

  //keyframe.shoulder = (shoulderPosition);
  //keyframe.bicep = (shoulderPosition);
  //keyframe.elbow = (elbowPosition);
  //keyframe.wrist = (wristPosition);
  //keyframe.fingerPosition = (fingerPosition);
  //keyframe.fingerRight = (fingerPlaneX);
  //keyframe.fingerForward = (fingerPlaneZ);

  return 0;
}


void robot_findHome() {
  motor_engage();

  char i;
  // for each stepper,
  for (i = 0; i < NUM_MOTORS; ++i) {
    Serial.print("Homing ");
    Serial.print(i);
    Serial.print('(');
    Serial.print(AxisNames[i]);
    Serial.println(')');

    // back up until all switches are hit
    digitalWrite(motors[i].dir_pin, HIGH);
    while ( digitalRead(motors[i].limit_switch_pin) == HIGH ) {
      // move "down"
      digitalWrite(motors[i].step_pin, HIGH);
      digitalWrite(motors[i].step_pin, LOW);
      pause(STEP_DELAY);
    }
    // back off in case we started in hit position
    digitalWrite(motors[i].dir_pin, LOW);
    while ( digitalRead(motors[i].limit_switch_pin) == LOW ) {
      // move "down"
      digitalWrite(motors[i].step_pin, HIGH);
      digitalWrite(motors[i].step_pin, LOW);
      pause(STEP_DELAY);
    }
    // back up until all switches are hit
    digitalWrite(motors[i].dir_pin, HIGH);
    while ( digitalRead(motors[i].limit_switch_pin) == HIGH ) {
      // move "down"
      digitalWrite(motors[i].step_pin, HIGH);
      digitalWrite(motors[i].step_pin, LOW);
      pause(STEP_DELAY);
    }
  }
  // set robot to home position
  float zeros[6] = {0, 0, 0, 0, 0, 0};
  teleport(zeros);
}


void robot_setup() {
  pinMode(MOTOR_0_LIMIT_SWITCH_PIN, INPUT);  digitalWrite(MOTOR_0_LIMIT_SWITCH_PIN, HIGH);
  pinMode(MOTOR_1_LIMIT_SWITCH_PIN, INPUT);  digitalWrite(MOTOR_1_LIMIT_SWITCH_PIN, HIGH);
  pinMode(MOTOR_2_LIMIT_SWITCH_PIN, INPUT);  digitalWrite(MOTOR_2_LIMIT_SWITCH_PIN, HIGH);
  pinMode(MOTOR_3_LIMIT_SWITCH_PIN, INPUT);  digitalWrite(MOTOR_3_LIMIT_SWITCH_PIN, HIGH);
  pinMode(MOTOR_4_LIMIT_SWITCH_PIN, INPUT);  digitalWrite(MOTOR_4_LIMIT_SWITCH_PIN, HIGH);
  pinMode(MOTOR_5_LIMIT_SWITCH_PIN, INPUT);  digitalWrite(MOTOR_5_LIMIT_SWITCH_PIN, HIGH);
}


#endif

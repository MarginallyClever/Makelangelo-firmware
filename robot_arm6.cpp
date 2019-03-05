  //------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#include "configure.h"
#include "robot_arm6.h"

#if MACHINE_STYLE == ARM6

#include "Vector3.h"

/**
   Inverse Kinematics turns XY coordinates into step counts from each motor
   This code is a duplicate of https://github.com/MarginallyClever/Robot-Overlord-App/blob/master/src/main/java/com/marginallyclever/robotOverlord/sixiRobot/java inverseKinematics()
   @param axies the cartesian coordinate
   @param motorStepArray a measure of each belt to that plotter position
*/
void IK(const float *const axies, long *motorStepArray) {
  // each of the xyz motors are differential to each other.
  // to move only one motor means applying the negative of that value to the other two motors

  // consider a two motor differential: 
  // if x moves, subtract x from y.
  // if y moves, subtract y from x.
  // so for three axis,
  // for any axis N subtract the other two axies from this axis.

  float J0=axies[0];  // hand (G0 X*)
  float J1=axies[1];  // wrist (G0 Y*)
  float J2=axies[2];  // ulna (G0 Z*)

  /*
  float x = a;//+b+c;  // supposed to move hand
  float y = b;//+c;  // supposed to move wrist
  float z = c;  // supposed to move ulna*/
  float J3 = axies[3];  // u ulna
  float J4 = axies[4];  // v wrist
  float J5 = axies[5];  // w hand

  // differential
  J5 += J4+J3;
  J4 += J3;
  
  motorStepArray[0] = J0 * MOTOR_0_STEPS_PER_TURN / 360.0;  // ANCHOR
  motorStepArray[1] = J1 * MOTOR_1_STEPS_PER_TURN / 360.0;  // SHOULDER
  motorStepArray[2] = J2 * MOTOR_2_STEPS_PER_TURN / 360.0;  // ELBOW
  motorStepArray[3] = J3 * MOTOR_3_STEPS_PER_TURN / 360.0;  // ULNA
  motorStepArray[4] = J4 * MOTOR_4_STEPS_PER_TURN / 360.0;  // WRIST
  motorStepArray[5] = J5 * MOTOR_5_STEPS_PER_TURN / 360.0;  // HAND
  motorStepArray[NUM_MOTORS] = axies[6];

  Serial.print("J=");  Serial.print(J0);
  Serial.print('\t');  Serial.print(J1);
  Serial.print('\t');  Serial.print(J2);
  Serial.print('\t');  Serial.print(J3);
  Serial.print('\t');  Serial.print(J4);
  Serial.print('\t');  Serial.print(J5);
  Serial.print('\n');
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
  /*
  float angle0rad = motorStepArray[0] * PI*2.0 / MOTOR_0_STEPS_PER_TURN;
  float angle1rad = motorStepArray[1] * PI*2.0 / MOTOR_1_STEPS_PER_TURN;
  float angle2rad = motorStepArray[2] * PI*2.0 / MOTOR_2_STEPS_PER_TURN;
  float angle3rad = motorStepArray[3] * PI*2.0 / MOTOR_3_STEPS_PER_TURN;
  float angle4rad = motorStepArray[4] * PI*2.0 / MOTOR_4_STEPS_PER_TURN;
  float angle5rad = motorStepArray[5] * PI*2.0 / MOTOR_5_STEPS_PER_TURN;*/

  // TODO fill me in!

  return 0;
}


void robot_findHome() {
  motor_engage();

  int homeDirections[NUM_MOTORS] = {HOME_DIR_0,HOME_DIR_1,HOME_DIR_2,HOME_DIR_3,HOME_DIR_4,HOME_DIR_5};
  
  uint8_t i;
  // for each stepper,
  for (i = 0; i < NUM_MOTORS; ++i) {
    Serial.print("Homing ");
    Serial.print(i,DEC);
    Serial.print('(');
    Serial.print(AxisNames[i]);
    Serial.println(')');

    // back up until switch is hit
    digitalWrite(motors[i].dir_pin, homeDirections[i]);
    while ( digitalRead(motors[i].limit_switch_pin) == HIGH ) {
      // move "down"
      digitalWrite(motors[i].step_pin, HIGH);
      digitalWrite(motors[i].step_pin, LOW);
      pause(STEP_DELAY);
    }
    // back off in case we started in hit position
    digitalWrite(motors[i].dir_pin, homeDirections[i]==HIGH?LOW:HIGH);
    while ( digitalRead(motors[i].limit_switch_pin) == LOW ) {
      // move "down"
      digitalWrite(motors[i].step_pin, HIGH);
      digitalWrite(motors[i].step_pin, LOW);
      pause(STEP_DELAY);
    }
    // back up until switch is hit
    digitalWrite(motors[i].dir_pin, homeDirections[i]);
    while ( digitalRead(motors[i].limit_switch_pin) == HIGH ) {
      // move "down"
      digitalWrite(motors[i].step_pin, HIGH);
      digitalWrite(motors[i].step_pin, LOW);
      pause(STEP_DELAY);
    }
  }
  // set robot to home position
  float homeSteps[6] = {
    -45  / MOTOR_0_STEPS_PER_TURN,
    0    / MOTOR_1_STEPS_PER_TURN,
    188  / MOTOR_2_STEPS_PER_TURN,
    0    / MOTOR_3_STEPS_PER_TURN,
    -90  / MOTOR_4_STEPS_PER_TURN,
    0    / MOTOR_5_STEPS_PER_TURN
    };

  // these are the correct angles in the computer model
  //float homeAxies[6] ={ -43.414, 0, 17.358, 0, 0, 172 };

  // these are temporary until IK is finished.
  float homeAxies[6] ={-45,0,188,0,-90,0};
  
  //FK(homeSteps,homeAxies);
  teleport(homeAxies);
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

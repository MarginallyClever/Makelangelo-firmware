  //------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
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
  float x = -axies[0];
  float y = -axies[1];
  float z = -axies[2];
  float u = -axies[3];
  float v =  axies[4];
  float w = -axies[5];
  
  motorStepArray[0] = x * MOTOR_0_STEPS_PER_TURN / 360.0;
  motorStepArray[1] = y * MOTOR_1_STEPS_PER_TURN / 360.0;
  motorStepArray[2] = z * MOTOR_2_STEPS_PER_TURN / 360.0;
  motorStepArray[3] = u * MOTOR_3_STEPS_PER_TURN / 360.0;
  motorStepArray[4] = v * MOTOR_4_STEPS_PER_TURN / 360.0;
  motorStepArray[5] = w * MOTOR_5_STEPS_PER_TURN / 360.0;
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

  // TODO fill me in!

  return 0;
}


void robot_findHome() {
  motor_engage();

  int homeDirections[NUM_MOTORS] = {HOME_DIR_0,HOME_DIR_1,HOME_DIR_2,HOME_DIR_3,HOME_DIR_4,HOME_DIR_5};
  
  char i;
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

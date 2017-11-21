//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == ARM6


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

  motorStepArray[0] = axies[0] * MOTOR_0_STEPS_PER_TURN / 360.0;
  motorStepArray[1] = axies[1] * MOTOR_1_STEPS_PER_TURN / 360.0;
  motorStepArray[2] = axies[2] * MOTOR_2_STEPS_PER_TURN / 360.0;
  motorStepArray[3] = axies[3] * MOTOR_3_STEPS_PER_TURN / 360.0;
  motorStepArray[4] = axies[4] * MOTOR_4_STEPS_PER_TURN / 360.0;
  motorStepArray[5] = axies[5] * MOTOR_5_STEPS_PER_TURN / 360.0;
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


void robot_findHome() {
  motor_engage();
  
  char i,hits;
  // back up until all switches are hit
  do {
    hits=0;
    // for each stepper,
    for(i=0;i<NUM_MOTORS;++i) {
      // if this switch hasn't been hit yet
      if( digitalRead(motors[i].limit_switch_pin) == HIGH ) {
        // move "down"
        digitalWrite(motors[i].dir_pin,LOW);
        digitalWrite(motors[i].step_pin,HIGH);
        digitalWrite(motors[i].step_pin,LOW);
      }
    }
    pause(STEP_DELAY);
  } while(hits>0);

  // set robot to home position
  float zeros[6] = {0,0,0,0,0,0};
  teleport(zeros);
}


void robot_setup() {
  pinMode(MOTOR_0_LIMIT_SWITCH_PIN,INPUT);
  pinMode(MOTOR_1_LIMIT_SWITCH_PIN,INPUT);
  pinMode(MOTOR_2_LIMIT_SWITCH_PIN,INPUT);
  pinMode(MOTOR_3_LIMIT_SWITCH_PIN,INPUT);
  pinMode(MOTOR_4_LIMIT_SWITCH_PIN,INPUT);
  pinMode(MOTOR_5_LIMIT_SWITCH_PIN,INPUT);

  digitalWrite(MOTOR_0_LIMIT_SWITCH_PIN,HIGH);
  digitalWrite(MOTOR_1_LIMIT_SWITCH_PIN,HIGH);
  digitalWrite(MOTOR_2_LIMIT_SWITCH_PIN,HIGH);
  digitalWrite(MOTOR_3_LIMIT_SWITCH_PIN,HIGH);
  digitalWrite(MOTOR_4_LIMIT_SWITCH_PIN,HIGH);
  digitalWrite(MOTOR_5_LIMIT_SWITCH_PIN,HIGH);
}


#endif

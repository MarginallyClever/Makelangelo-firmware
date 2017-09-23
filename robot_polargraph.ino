//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#if MACHINE_STYLE == POLARGRAPH


/**
 * Inverse Kinematics turns XY coordinates into step counts from each motor
 * @param axies the cartesian coordinate
 * @param motorStepArray a measure of each belt to that plotter position
 */
void IK(float *cartesian, long *motorStepArray) {
  float dy,dx;
  // find length to M1
  float limit_xmin = axies[0].limitMin;
  float limit_xmax = axies[0].limitMax;
  float limit_ymax = axies[1].limitMax;
  
  dy = cartesian[1] - limit_ymax;
  dx = cartesian[0] - limit_xmin;
  motorStepArray[0] = lround( sqrt(dx*dx+dy*dy) / THREAD_PER_STEP );
  // find length to M2
  dx = limit_xmax - cartesian[0];
  motorStepArray[1] = lround( sqrt(dx*dx+dy*dy) / THREAD_PER_STEP );

  motorStepArray[NUM_MOTORS] = cartesian[2];
}


/** 
 * Forward Kinematics - turns step counts into XY coordinates
 * @param motorStepArray a measure of each belt to that plotter position
 * @param axies the resulting cartesian coordinate
 * @return 0 if no problem, 1 on failure.
 */
int FK(long *motorStepArray,float *cartesian) {
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



void recordHome() {
#ifdef USE_LIMIT_SWITCH
  wait_for_empty_segment_buffer();
  motor_engage();
  findStepDelay();

  Serial.println(F("Record home..."));

  digitalWrite(MOTOR_0_DIR_PIN, LOW);
  digitalWrite(MOTOR_1_DIR_PIN, LOW);
  int left = 0;
  int right = 0;
  long count[NUM_MOTORS];

  // we start at home position, so we know (x,y)->(left,right) value here.
  IK(homeX, homeY, count);
  Serial.print(F("HX="));  Serial.println(homeX);
  Serial.print(F("HY="));  Serial.println(homeY);
  //Serial.print(F("L1="));  Serial.println(leftCount);
  //Serial.print(F("R1="));  Serial.println(rightCount);

  Serial.println(F("A..."));
  do {
    if (left == 0) {
      if ( digitalRead(LIMIT_SWITCH_PIN_LEFT) == LOW ) {
        left = 1;
        Serial.println(F("Left..."));
      }
      ++count[0];
      digitalWrite(MOTOR_0_STEP_PIN, HIGH);
      digitalWrite(MOTOR_0_STEP_PIN, LOW);
    }
    if (right == 0) {
      if ( digitalRead(LIMIT_SWITCH_PIN_RIGHT) == LOW ) {
        right = 1;
        Serial.println(F("Right..."));
      }
      ++count[1];
      digitalWrite(MOTOR_1_STEP_PIN, HIGH);
      digitalWrite(MOTOR_1_STEP_PIN, LOW);
    }
    pause(step_delay * 2);
  } while (left + right < 2);

  Serial.println(F("B..."));
  digitalWrite(MOTOR_0_DIR_PIN, HIGH);
  digitalWrite(MOTOR_1_DIR_PIN, HIGH);
  for (int i = 0; i < STEPS_PER_TURN; ++i) {
    digitalWrite(MOTOR_0_STEP_PIN, HIGH);
    digitalWrite(MOTOR_0_STEP_PIN, LOW);
    digitalWrite(MOTOR_1_STEP_PIN, HIGH);
    digitalWrite(MOTOR_1_STEP_PIN, LOW);
    pause(step_delay * 4);
    --count[0];
    --count[1];
  }

  left = right = 0;
  Serial.println(F("C..."));
  digitalWrite(MOTOR_0_DIR_PIN, LOW);
  digitalWrite(MOTOR_1_DIR_PIN, LOW);
  do {
    if (left == 0) {
      if ( digitalRead(LIMIT_SWITCH_PIN_LEFT) == LOW ) {
        left = 1;
        Serial.println(F("Left..."));
      }
      ++count[0];
      digitalWrite(MOTOR_0_STEP_PIN, HIGH);
      digitalWrite(MOTOR_0_STEP_PIN, LOW);
    }
    if (right == 0) {
      if ( digitalRead(LIMIT_SWITCH_PIN_RIGHT) == LOW ) {
        right = 1;
        Serial.println(F("Right..."));
      }
      ++count[1];
      digitalWrite(MOTOR_1_STEP_PIN, HIGH);
      digitalWrite(MOTOR_1_STEP_PIN, LOW);
    }
    pause(step_delay * 4);
  } while (left + right < 2);

  calibrateLeft = count[0];
  calibrateRight = count[1];

  // now we have the count from home position to switches.  record that value.
  saveCalibration();
  reportCalibration();

  // current position is...
  float axies[NUM_AXIES];
  FK(count, axies);
  teleport(axies[0],axies[1],axies[2]);
  where();

  // go home.
  Serial.println(F("Homing..."));

  Vector3 offset = get_end_plus_offset();
  line_safe(homeX, homeY, offset.z, feed_rate);
  Serial.println(F("Done."));
#endif // USER_LIMIT_SWITCH
}


/**
   If limit switches are installed, move to touch each switch so that the pen holder can move to home position.
*/
void robot_findHome() {
#ifdef USE_LIMIT_SWITCH
  wait_for_empty_segment_buffer();
  motor_engage();

  Serial.println(F("Find Home..."));

  findStepDelay();

  // reel in the left motor and the right motor out until contact is made.
  digitalWrite(MOTOR_0_DIR_PIN, LOW);
  digitalWrite(MOTOR_1_DIR_PIN, LOW);
  int left = 0, right = 0;
  do {
    if (left == 0) {
      if ( digitalRead(LIMIT_SWITCH_PIN_LEFT) == LOW ) {
        left = 1;
        Serial.println(F("Left..."));
      }
      digitalWrite(MOTOR_0_STEP_PIN, HIGH);
      digitalWrite(MOTOR_0_STEP_PIN, LOW);
    }
    if (right == 0) {
      if ( digitalRead(LIMIT_SWITCH_PIN_RIGHT) == LOW ) {
        right = 1;
        Serial.println(F("Right..."));
      }
      digitalWrite(MOTOR_1_STEP_PIN, HIGH);
      digitalWrite(MOTOR_1_STEP_PIN, LOW);
    }
    pause(step_delay);
  } while (left + right < 2);

  // make sure there's no momentum to skip the belt on the pulley.
  delay(500);

  Serial.println(F("Estimating position..."));
  long count[NUM_MOTORS];
  count[0] = calibrateLeft;
  count[1] = calibrateRight;
  Serial.print("cl=");   Serial.println(calibrateLeft);
  Serial.print("cr=");   Serial.println(calibrateRight);
  Serial.print("t=");    Serial.println(THREAD_PER_STEP);

  // current position is...
  float axies[NUM_AXIES];
  FK(count, axies);
  teleport(axies[0],axies[1],axies[2]);
  where();

  // go home.
  Serial.println(F("Homing..."));

  Vector3 offset = get_end_plus_offset();
  line_safe(homeX, homeY, offset.z, feed_rate);
  Serial.println(F("Done."));
#endif // USER_LIMIT_SWITCH
}


/**
 * Starting from the home position, bump the switches and measure the length of each belt.
 * Does not save the values, only reports them to serial.
 */
void calibrateBelts() {
#ifdef USE_LIMIT_SWITCH
  wait_for_empty_segment_buffer();
  motor_engage();

  Serial.println(F("Find switches..."));

  // reel in the left motor and the right motor out until contact is made.
  digitalWrite(MOTOR_0_DIR_PIN, LOW);
  digitalWrite(MOTOR_1_DIR_PIN, LOW);
  int left = 0, right = 0;
  long steps[NUM_MOTORS];
  float homePos[NUM_AXIES];
  homePos[0]=homeX;
  homePos[1]=homeY;
  homePos[2]=posz;
  IK(homePos, steps);
  findStepDelay();

  do {
    if (left == 0) {
      if ( digitalRead(LIMIT_SWITCH_PIN_LEFT ) == LOW ) {
        // switch hit
        left = 1;
      }
      // switch not hit yet, keep moving
      digitalWrite(MOTOR_0_STEP_PIN, HIGH);
      digitalWrite(MOTOR_0_STEP_PIN, LOW);
      steps[0]++;
    }
    if (right == 0) {
      if ( digitalRead(LIMIT_SWITCH_PIN_RIGHT ) == LOW ) {
        // switch hit
        right = 1;
      }
      // switch not hit yet, keep moving
      digitalWrite(MOTOR_1_STEP_PIN, HIGH);
      digitalWrite(MOTOR_1_STEP_PIN, LOW);
      steps[1]++;
    }
    pause(step_delay);
  } while (left + right < NUM_MOTORS);

  // make sure there's no momentum to skip the belt on the pulley.
  delay(500);

  Serial.println(F("Estimating position..."));
  calibrateLeft  = (float)steps[0] * THREAD_PER_STEP;
  calibrateRight = (float)steps[1] * THREAD_PER_STEP;

  reportCalibration();

  // current position is...
  float axies[NUM_AXIES];
  FK(steps, axies);
  teleport(axies[0],axies[1],axies[2]);
  where();

  // go home.
  Serial.println(F("Homing..."));

  Vector3 offset = get_end_plus_offset();
  line_safe(homeX, homeY, offset.z, feed_rate);
  Serial.println(F("Done."));
#endif // USE_LIMIT_SWITCH
}

#endif  // POLARGRAPH

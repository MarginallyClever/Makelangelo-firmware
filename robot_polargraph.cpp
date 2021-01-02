//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#include "configure.h"

#if MACHINE_STYLE == POLARGRAPH

/**
 * Inverse Kinematics turns XY coordinates into step counts from each motor
 * @param axies the cartesian coordinate
 * @param motorStepArray a measure of each belt to that plotter position
 */
void IK(const float *const cartesian, long *motorStepArray) {
  float dy,dx;
  // find length to M1
  float left = axies[0].limitMin;
  float right = axies[0].limitMax;
  float top = axies[1].limitMax;
  
  dy = cartesian[1] - top;
  dx = cartesian[0] - left;
  motorStepArray[0] = lroundf( sqrt(sq(dx)+sq(dy)) / MM_PER_STEP );
  // find length to M2
  dx = right - cartesian[0];
  motorStepArray[1] = lroundf( sqrt(sq(dx)+sq(dy)) / MM_PER_STEP );
  
  motorStepArray[2] = cartesian[2];
}


/** 
 * Forward Kinematics - turns step counts into XY coordinates
 * @param motorStepArray a measure of each belt to that plotter position.  NUM_MUSCLES long.
 * @param axies the resulting cartesian coordinate. NUM_AXIES long.
 * @return 0 if no problem, 1 on failure.
 */
int FK(long *motorStepArray,float *cartesian) {
  float left = axies[0].limitMin;
  float right = axies[0].limitMax;
  float top = axies[1].limitMax;
  
  // use law of cosines: theta = acos((a*a+b*b-c*c)/(2*a*b));
  float a = (float)motorStepArray[0] * MM_PER_STEP;
  float b = (right-left);
  float c = (float)motorStepArray[1] * MM_PER_STEP;

  // slow, uses trig
  // we know law of cosines:   cc = aa + bb -2ab * cos( theta )
  // or cc - aa - bb = -2ab * cos( theta )
  // or ( aa + bb - cc ) / ( 2ab ) = cos( theta );
  // or theta = acos((aa+bb-cc)/(2ab));
  // so  x = cos(theta)*l1 + left;
  // and y = sin(theta)*l1 + top;
  // and we know that cos(acos(i)) = i
  // and we know that sin(acos(i)) = sqrt(1-i*i)
  // so y = sin(  acos((aa+bb-cc)/(2ab))  )*l1 + top;
  float theta = ((a*a+b*b-c*c)/(2.0*a*b));
  
  cartesian[0] = theta * a + left;
  /*
  Serial.print("ymax=");   Serial.println(top);
  Serial.print("theta=");  Serial.println(theta);
  Serial.print("a=");      Serial.println(a);
  Serial.print("b=");      Serial.println(b);
  Serial.print("c=");      Serial.println(c);
  Serial.print("S0=");     Serial.println(motorStepArray[0]);
  Serial.print("S1=");     Serial.println(motorStepArray[1]);
  */
  cartesian[1] = top - sqrt( 1.0 - theta * theta ) * a;
  cartesian[2] = motorStepArray[2];
  /*
  Serial.print("C0=");      Serial.println(cartesian[0]);
  Serial.print("C1=");      Serial.println(cartesian[1]);
  Serial.print("C2=");      Serial.println(cartesian[2]);
  */
  return 0;
}



void recordHome() {
#if defined(CAN_HOME)
  wait_for_empty_segment_buffer();
  motor_engage();
#ifdef MACHINE_HAS_LIFTABLE_PEN
  setPenAngle(PEN_UP_ANGLE);
#endif
  findStepDelay();

  Serial.println(F("Record home..."));

  digitalWrite(MOTOR_0_DIR_PIN, LOW);
  digitalWrite(MOTOR_1_DIR_PIN, LOW);
  int left = 0;
  int right = 0;
  long count[NUM_MUSCLES];

  // we start at home position, so we know (x,y)->(left,right) value here.
  float homes[NUM_AXIES];
  homes[0]=axies[0].homePos;
  homes[1]=axies[1].homePos;
  homes[2]=axies[2].pos;
  IK(homes, count);
  Serial.print(F("HX="));  Serial.println(homes[0]);
  Serial.print(F("HY="));  Serial.println(homes[1]);
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
  eepromManager.saveCalibration();
  reportCalibration();

  // current position is...
  float offset[NUM_AXIES];
  FK(count, offset);
  teleport(offset);
  parser.M114();
  Serial.println(F("Done."));
#endif // defined(CAN_HOME)
}

void polargraph_homeAtSpeed(int delayTime) {
  // reel in the left motor and the right motor out until contact is made.
  digitalWrite(MOTOR_0_DIR_PIN, STEPPER_DIR_LOW);
  digitalWrite(MOTOR_1_DIR_PIN, STEPPER_DIR_LOW);
  int left = 0, right = 0;
  do {
    if (left == 0) {
      digitalWrite(MOTOR_0_STEP_PIN, HIGH);
      digitalWrite(MOTOR_0_STEP_PIN, LOW);
#ifdef USE_LIMIT_SWITCH
      if ( digitalRead(LIMIT_SWITCH_PIN_LEFT) == LOW )
#endif
      {
        left = 1;
      }
    }
    if (right == 0) {
      digitalWrite(MOTOR_1_STEP_PIN, HIGH);
      digitalWrite(MOTOR_1_STEP_PIN, LOW);
#ifdef USE_LIMIT_SWITCH
      if ( digitalRead(LIMIT_SWITCH_PIN_RIGHT) == LOW )
#endif
      {
        right = 1;
      }
    }
    pause(delayTime);
  } while (left + right < 2);
}


/**
 *  If limit switches are installed, move to touch each switch so that the pen holder can move to home position.
 */
void robot_findHome() {
#if defined(CAN_HOME)
  // do not run this code unless you have the hardware to find home!
  wait_for_empty_segment_buffer();
  motor_engage();
#ifdef MACHINE_HAS_LIFTABLE_PEN
  setPenAngle(PEN_UP_ANGLE);
#endif

  Serial.println(F("Find Home..."));

  #ifdef HAS_TMC2130
  	delay(500);
  	digitalWrite(MOTOR_0_DIR_PIN, STEPPER_DIR_HIGH);
  	digitalWrite(MOTOR_1_DIR_PIN, STEPPER_DIR_HIGH);
  
  	motor_home();
    
  	while(homing == true){
  		Serial.print(driver_0.TSTEP());
  		Serial.print("   ");
  		Serial.print(digitalRead(MOTOR_0_LIMIT_SWITCH_PIN));
  		Serial.print("   ");
  		Serial.print(digitalRead(MOTOR_1_LIMIT_SWITCH_PIN));
  		Serial.print("   ");
  	  Serial.println("still homing");
  	}
  	Serial.println("BOTH EN false");
  	enable_stealthChop();
  
  #else

    findStepDelay();
    polargraph_homeAtSpeed(step_delay);
    // make sure there's no momentum to skip the belt on the pulley.
    delay(500);
    // back off a bit
    digitalWrite(MOTOR_0_DIR_PIN, STEPPER_DIR_HIGH);
    digitalWrite(MOTOR_1_DIR_PIN, STEPPER_DIR_HIGH);
    for(int i=0;i<500;++i) {
        digitalWrite(MOTOR_0_STEP_PIN, HIGH);
        digitalWrite(MOTOR_0_STEP_PIN, LOW);
      
        digitalWrite(MOTOR_1_STEP_PIN, HIGH);
        digitalWrite(MOTOR_1_STEP_PIN, LOW);
        pause(step_delay*3);
    }
    // find it again, but slower
    polargraph_homeAtSpeed(step_delay*10);
    
  #endif  // HAS_TMC2130
  
  //Serial.println(F("Estimating position..."));
  long count[NUM_MUSCLES];
  count[0] = calibrateLeft/MM_PER_STEP;
  count[1] = calibrateRight/MM_PER_STEP;
  count[2] = axies[2].pos;
  //Serial.print("cl=");        Serial.println(calibrateLeft);
  //Serial.print("cr=");        Serial.println(calibrateRight);
  //Serial.print("t*1000=");    Serial.println(MM_PER_STEP*1000);

  // current position is...
  float offset[NUM_AXIES];
  FK(count, offset);
  teleport(offset);
  parser.M114();

  // go home
  float pos[NUM_AXIES];
  offset[0]=axies[0].homePos;
  offset[1]=axies[1].homePos;
  lineSafe( offset, feed_rate );
  
  Serial.println(F("Done."));
#endif // defined(CAN_HOME)
}


/**
 * Starting from the home position, bump the switches and measure the length of each belt.
 * Does not save the values, only reports them to serial.
 * @Deprecated
 */
void calibrateBelts() {
#if defined(CAN_HOME)
  wait_for_empty_segment_buffer();
  motor_engage();
#ifdef MACHINE_HAS_LIFTABLE_PEN
  setPenAngle(PEN_UP_ANGLE);
#endif

  Serial.println(F("Find switches..."));

  // reel in the left motor and the right motor out until contact is made.
  digitalWrite(MOTOR_0_DIR_PIN, LOW);
  digitalWrite(MOTOR_1_DIR_PIN, LOW);
  int left = 0, right = 0;
  long steps[NUM_MUSCLES];
  float homePos[NUM_AXIES];
  homePos[0]=axies[0].homePos;
  homePos[1]=axies[1].homePos;
  homePos[2]=axies[2].pos;
  IK(homePos, steps);
  findStepDelay();

  do {
    if (left == 0) {
      if ( digitalRead(LIMIT_SWITCH_PIN_LEFT) == LOW ) {
        // switch hit
        left = 1;
      }
      // switch not hit yet, keep moving
      digitalWrite(MOTOR_0_STEP_PIN, HIGH);
      digitalWrite(MOTOR_0_STEP_PIN, LOW);
      steps[0]++;
    }
    if (right == 0) {
      if ( digitalRead(LIMIT_SWITCH_PIN_RIGHT) == LOW ) {
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
  calibrateLeft  = (float)steps[0] * MM_PER_STEP;
  calibrateRight = (float)steps[1] * MM_PER_STEP;

  reportCalibration();

  // current position is...
  float axies2[NUM_AXIES];
  FK(steps, axies2);
  teleport(axies2);
  parser.M114();

  // go home.
  Serial.println(F("Homing..."));

  float offset[NUM_AXIES];
  get_end_plus_offset(offset);
  offset[0]=axies[0].homePos;
  offset[1]=axies[1].homePos;
  lineSafe(offset, feed_rate);
  Serial.println(F("Done."));
#endif // defined(CAN_HOME)
}


// convert belt length to cartesian position, save that as home pos.
void calibrationToPosition() {
  float axies2[NUM_AXIES];
  long steps[3];
  steps[0]=calibrateLeft;
  steps[1]=calibrateRight;
  steps[2]=axies[2].pos;
  FK(steps, axies2);
    
  teleport(axies2);
}

/**
 * makelangelo 6 specific setup call
 */
inline void polargraphResetM6() {
  // if you accidentally upload m3 firmware to an m5 then upload it ONCE with this line uncommented.
  float limits[NUM_AXIES * 2];
  limits[0] = 707.5 / 2;
  limits[1] = -707.5 / 2;
  limits[2] = 500;
  limits[3] = -500;
  limits[4] = PEN_UP_ANGLE;
  limits[5] = PEN_DOWN_ANGLE;
  eepromManager.adjustLimits(limits);

  calibrateLeft = 1025;
  calibrateRight = 1025;
  eepromManager.saveCalibration();
  calibrationToPosition();
  
  // set home
  float homePos[NUM_AXIES];
  homePos[0] = 0;
  homePos[1] = 0;
  homePos[2] = 90;
  setHome(homePos);
}


/**
 * makelangelo 5 specific setup call
 */
inline void polargraphResetM5() {
  // if you accidentally upload m3 firmware to an m5 then upload it ONCE with this line uncommented.
  float limits[NUM_AXIES * 2];
  limits[0] = 325.0;
  limits[1] = -325.0;
  limits[2] = 500;
  limits[3] = -500;
  limits[4] = PEN_UP_ANGLE;
  limits[5] = PEN_DOWN_ANGLE;
  eepromManager.adjustLimits(limits);

  calibrateLeft = 1025;
  calibrateRight = 1025;
  eepromManager.saveCalibration();
  calibrationToPosition();
  
  // set home
  float homePos[NUM_AXIES];
  homePos[0] = 0;
  homePos[1] = 0;
  homePos[2] = 90;
  setHome(homePos);
}


/**
 * makelangelo 3.3 specific setup call
 */
inline void polargraphResetM33() {
  float limits[NUM_AXIES * 2];
  limits[0] = 1000.0;
  limits[1] = -1000.0;
  limits[2] = 800;
  limits[3] = -800;
  limits[4] = PEN_UP_ANGLE;
  limits[5] = PEN_DOWN_ANGLE;
  eepromManager.adjustLimits(limits);

  calibrateLeft = 2022;
  calibrateRight = 2022;
  eepromManager.saveCalibration();
  calibrationToPosition();
  
  // set home
  float homePos[NUM_AXIES];
  homePos[0] = 0;
  homePos[1] = 0;
  homePos[2] = 90;
  setHome(homePos);
}


/**
 * M503 factory reset
 */
void polargraphReset() {
#if MACHINE_HARDWARE_VERSION == MAKELANGELO_6
  polargraphResetM6();
#endif
#if MACHINE_HARDWARE_VERSION == MAKELANGELO_5
  polargraphResetM5();
#endif
#if MACHINE_HARDWARE_VERSION == MAKELANGELO_3_3
  polargraphResetM33();
#endif
}

// called once at startup
void robot_setup() {
}


#endif  // POLARGRAPH

//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#include "configure.h"

#if MACHINE_STYLE == SIXI

#include "Vector3.h"

//#define DEBUG_IK

char sensorPins[4*NUM_SENSORS];
float sensorAngles[NUM_SENSORS];

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

  // Some of these are negative because the motor is wired to turn the opposite direction from the Robot Overlord simulation.
  // Robot Overlord has the final say, so these are flipped to match the simulation.
  // This is the only place motor direction should ever be inverted.
  float J0 = -axies[0];  // anchor  (G0 X*)
  float J1 =  axies[1];  // shoulder (G0 Y*)
  float J2 =  axies[2];  // elbow (G0 Z*)
  float J3 = -axies[3];  // ulna  (G0 U*)
  float J4 =  axies[4];  // wrist (G0 V*)
  float J5 = -axies[5];  // hand  (G0 W*)

  float j4Adjust = -(J3/NEMA17_CYCLOID_GEARBOX_RATIO);
  float j5Adjust = (J4/NEMA17_CYCLOID_GEARBOX_RATIO)+j4Adjust;

  // adjust for the wrist differential
  J4 += j4Adjust;
  J5 += j5Adjust;
  
  
  motorStepArray[0] = J0 * MOTOR_0_STEPS_PER_TURN / 360.0;  // ANCHOR
  motorStepArray[1] = J1 * MOTOR_1_STEPS_PER_TURN / 360.0;  // SHOULDER
  motorStepArray[2] = J2 * MOTOR_2_STEPS_PER_TURN / 360.0;  // ELBOW
  motorStepArray[3] = J3 * MOTOR_3_STEPS_PER_TURN / 360.0;  // ULNA
  motorStepArray[4] = J4 * MOTOR_4_STEPS_PER_TURN / 360.0;  // WRIST
  motorStepArray[5] = J5 * MOTOR_5_STEPS_PER_TURN / 360.0;  // HAND
  motorStepArray[6] = axies[6];
  
#ifdef DEBUG_IK
  Serial.print("J=");  Serial.print(J0);
  Serial.print('\t');  Serial.print(J1);
  Serial.print('\t');  Serial.print(J2);
  Serial.print('\t');  Serial.print(J3);
  Serial.print('\t');  Serial.print(J4);
  Serial.print('\t');  Serial.print(J5);
  Serial.print('\n');
#endif
}


/**
   Forward Kinematics - turns step counts into XY coordinates.  
   This code is a duplicate of https://github.com/MarginallyClever/Robot-Overlord-App/blob/master/src/main/java/com/marginallyclever/robotOverlord/sixiRobot/java forwardKinematics()
   @param motorStepArray a measure of each belt to that plotter position
   @param axies the resulting cartesian coordinate
   @return 0 if no problem, 1 on failure.
*/
int FK(long *motorStepArray, float *axies) {
  // TODO fill me in!

  return 0;
}


void robot_findHome() {
  motor_engage();
  // sixi always knows where it is.
  float pos[NUM_AXIES] = {0,-90,0,0,20,0,0};
  lineSafe(pos,feed_rate);
}


void robot_setup() {
  int i=0;

#define SSP(label,NN)    sensorPins[i++] = PIN_SENSOR_##label##_##NN;
#define SSP2(NN)         if(NUM_SENSORS>NN) {  SSP(CSEL,NN)  SSP(CLK,NN)  SSP(MISO,NN)  SSP(MOSI,NN)  }

  SSP2(0)
  SSP2(1)
  SSP2(2)
  SSP2(3)
  SSP2(4)
  SSP2(5)

  for(ALL_SENSORS(i)) {
    pinMode(sensorPins[(i*4)+0],OUTPUT);  // csel
    pinMode(sensorPins[(i*4)+1],OUTPUT);  // clk
    pinMode(sensorPins[(i*4)+2],INPUT);  // miso
    pinMode(sensorPins[(i*4)+3],OUTPUT);  // mosi

    digitalWrite(sensorPins[(i*4)+0],HIGH);  // csel
    digitalWrite(sensorPins[(i*4)+3],HIGH);  // mosi
  }

  // slow the servo on pin D13 down to 61.04Hz
  // see https://arduinoinfo.mywikis.net/wiki/Arduino-PWM-Frequency
  //TCCR0B = (TCCR0B & B11111000) | B00000101;

  // the first few reads will return junk so we force a couple empties here.
  sensorUpdate();
  sensorUpdate();

  // initialize the step count to the sensor reading.
  parser.D18();
}

/**
 * @param index the sensor to read
 * @param result where to store the returned value.  may be changed even if method fails.
 * @return 0 on fail, 1 on success.
// @see https://ams.com/documents/20143/36005/AS5147_DS000307_2-00.pdf
 */
boolean getSensorRawValue(int index, uint16_t &result) {
  result=0;
  uint8_t input,parity=0;

  index*=4;
  
  // Send the request for the angle value (command 0xFFFF)
  // at the same time as receiving an angle.

  // Collect the 16 bits of data from the sensor
  digitalWrite(sensorPins[index+0],LOW);  // csel
  
  for(int i=0;i<SENSOR_TOTAL_BITS;++i) {
    digitalWrite(sensorPins[index+1],HIGH);  // clk
    // this is here to give a little more time to the clock going high.
    // only needed if the arduino is *very* fast.  I'm feeling generous.
    result <<= 1;
    digitalWrite(sensorPins[index+1],LOW);  // clk
    
    input = digitalRead(sensorPins[index+2]);  // miso
#ifdef VERBOSE
    Serial.print(input,DEC);
#endif
    result |= input;
    parity ^= (i>0) & input;
  }

  digitalWrite(sensorPins[index+0],HIGH);  // csel
  
  // check the parity bit
  return ( parity != (result>>SENSOR_DATA_BITS) );
}


/**
 * @param rawValue 16 bit value from as4157 sensor, including parity and EF bit
 * @return degrees calculated from bottom 14 bits.
 */
float extractAngleFromRawValue(uint16_t rawValue) {
  return (float)(rawValue & BOTTOM_14_MASK) * 360.0 / (float)(1<<SENSOR_ANGLE_BITS);
}


void sensorUpdate() {
  uint16_t rawValue;
  float v;
  for(ALL_SENSORS(i)) {
    if(getSensorRawValue(i,rawValue)) continue;
    v = extractAngleFromRawValue(rawValue);
    if(i!=1 && i!=2) v=-v;
    v -= axies[i].homePos;
    v = WRAP_DEGREES(v);
    sensorAngles[i] = v;
  }
}


void reportError() {
  wait_for_empty_segment_buffer();
  sensorUpdate();
    
  Serial.print(F("DP"));

  for(ALL_SENSORS(i)) {
    Serial.print('\t');
    Serial.print(AxisNames[i]);
    float dp = axies[i].pos-sensorAngles[i];
    Serial.print(dp, 3);
  }
  Serial.println();
}


void printGoto(float *pos) {
  Serial.print(F("G0"));
  for(ALL_AXIES(i)) {
    Serial.print('\t');
    Serial.print(AxisNames[i]);
    Serial.print(pos[i], 3);
  }
  Serial.println();
}


void drive(int i,int totalSteps,int t) {
  for(int j=0;j<totalSteps;++j) {
    digitalWrite(motors[i].step_pin,HIGH);
    digitalWrite(motors[i].step_pin,LOW);
    delay(t);
  }
}


void drive2(int i,float change) {
  float pos[NUM_AXIES] = {0,-90,0,0,20,0,0};
  
  pos[i] +=- change;
  
  printGoto(pos);
  lineSafe(pos,feed_rate);
}


void sixiDemo3a(int i,float t) {
  delay(100);

  int totalSteps=3000;

  digitalWrite(motors[i].dir_pin,HIGH);
  //drive(i,3000,t);
  drive2(i,-90);
  
  sensorUpdate();
  float uStart = sensorAngles[3];
  float vStart = sensorAngles[4];
  float wStart = sensorAngles[5];
  
  Serial.print(AxisNames[i]);
  Serial.print('s');
  Serial.print('\t');    Serial.print(uStart,5);
  Serial.print('\t');    Serial.print(vStart,5);
  Serial.print('\t');    Serial.println(wStart,5);
  
  digitalWrite(motors[i].dir_pin,LOW);
  //drive(i,6000,t);
  drive2(i,90);
  
  sensorUpdate();
  float uEnd = sensorAngles[3];
  float vEnd = sensorAngles[4];
  float wEnd = sensorAngles[5];
  
  Serial.print(AxisNames[i]);
  Serial.print('e');
  Serial.print('\t');    Serial.print(uEnd,5);
  Serial.print('\t');    Serial.print(vEnd,5);
  Serial.print('\t');    Serial.println(wEnd,5);

  Serial.print(AxisNames[i]);
  Serial.print('d');
  Serial.print('\t');    Serial.print((uEnd-uStart),5);
  Serial.print('\t');    Serial.print((vEnd-vStart),5);
  Serial.print('\t');    Serial.println((wEnd-wStart),5);

  // put it back where we found it
  digitalWrite(motors[i].dir_pin,HIGH);
  //drive(i,3000,t);
  drive2(i,0);
  
  Serial.println();
}


// move each axis and measure the degree-per-step of just that one axis.
void sixiDemo3() {
  robot_findHome();

  float of = feed_rate;
  feed_rate=60;
  
  Serial.println("Jn\tU\tV\tW");
  
  for(int i=3;i<6;++i) {
    sixiDemo3a(i,5);  
  }

  feed_rate=of;
}


void sixiDemo2a(float t) {
  delay(100);

  int totalSteps=200;

  Serial.print(t);
  
  for(ALL_SENSORS(i)) {
    digitalWrite(motors[i].dir_pin,LOW);
    sensorUpdate();
    float aStart = sensorAngles[i];
    for(int j=0;j<totalSteps;++j) {
      digitalWrite(motors[i].step_pin,HIGH);
      digitalWrite(motors[i].step_pin,LOW);
      delay(t);
    }
    sensorUpdate();
    float aEnd = sensorAngles[i];
    float perStep = fabs(aEnd-aStart)/(float)totalSteps;
    
    Serial.print('\t');
    Serial.print(perStep,5);

    // put it back where we found it
    digitalWrite(motors[i].dir_pin,HIGH);
    for(int j=0;j<totalSteps;++j) {
      digitalWrite(motors[i].step_pin,HIGH);
      digitalWrite(motors[i].step_pin,LOW);
      delay(t);
    }
  }
  Serial.println();
}


// move each axis and measure the degree-per-step of just that one axis.
void sixiDemo2() {
  robot_findHome();
  
  Serial.print("\ndt");
  
#define PPX(NN)  { Serial.print('\t');  Serial.print(AxisNames[NN]);  Serial.print(DEGREES_PER_STEP_##NN,5);  }
  PPX(0);
  PPX(1);
  PPX(2);
  PPX(3);
  PPX(4);
  PPX(5);
  Serial.println();
  
  for(int i=50;i>=20;--i) {
    sixiDemo2a(i);  
  }
}


void sixiDemo1() {
  Serial.println(F("SIXI DEMO START"));
  Serial.print("AXIES=");
  Serial.println(NUM_AXIES);
  
  float posHome[NUM_AXIES] = {0,-90,0,0,20,0,0};
  float pos[NUM_AXIES] = {0,0,0,0,0,0,0};
  int i,j;

  float fr = feed_rate;
  float aa = acceleration;

  feed_rate = 80;
  acceleration = 25;

  printGoto(posHome);
  robot_findHome();
  reportError();
  
  // one joint at a time
  for(ALL_AXIES(i)) {
    for(j=0;j<15;++j) 
    {
      for(ALL_AXIES(k)) {
        pos[k] = posHome[k];
      }
      pos[i] = posHome[i] - 10;
      
      printGoto(pos);
      lineSafe(pos,feed_rate);

      for(ALL_AXIES(k)) {
        pos[k] = posHome[k];
      }
      pos[i] = posHome[i] + 10;
      printGoto(pos);
      lineSafe(pos,feed_rate);
    }
    printGoto(posHome);
    robot_findHome();
    reportError();
  }
  
  // combo moves
  for(j=0;j<30;++j)
  {
    for(ALL_AXIES(i)) {
      pos[i] = posHome[i] + random(-10,10);
    }
    printGoto(pos);
    lineSafe(pos,feed_rate);
  }
  printGoto(posHome);
  robot_findHome();
  reportError();
  
  feed_rate = fr;
  acceleration = aa;
  
  Serial.println(F("SIXI DEMO END"));
}


//d15
void sixiDemo() {
  //sixiDemo1();
  //sixiDemo2();
  sixiDemo3();
}


#endif

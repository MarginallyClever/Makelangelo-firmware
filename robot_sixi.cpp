  //------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#include "configure.h"
#include "robot_sixi.h"

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

  // adjust for the wrist differential
  J5 += (J4/NEMA17_CYCLOID_GEARBOX_RATIO)+(J3/NEMA17_CYCLOID_GEARBOX_RATIO);
  J4 += (J3/NEMA17_CYCLOID_GEARBOX_RATIO);
  
  motorStepArray[0] = J0 * MOTOR_0_STEPS_PER_TURN / 360.0;  // ANCHOR
  motorStepArray[1] = J1 * MOTOR_1_STEPS_PER_TURN / 360.0;  // SHOULDER
  motorStepArray[2] = J2 * MOTOR_2_STEPS_PER_TURN / 360.0;  // ELBOW
  motorStepArray[3] = J3 * MOTOR_3_STEPS_PER_TURN / 360.0;  // ULNA
  motorStepArray[4] = J4 * MOTOR_4_STEPS_PER_TURN / 360.0;  // WRIST
  motorStepArray[5] = J5 * MOTOR_5_STEPS_PER_TURN / 360.0;  // HAND
  motorStepArray[NUM_MOTORS] = axies[6];
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
}


void robot_setup() {
  int i=0;

  sensorPins[i++]=PIN_SENSOR_CSEL_0;
  sensorPins[i++]=PIN_SENSOR_CLK_0;
  sensorPins[i++]=PIN_SENSOR_MISO_0;
  sensorPins[i++]=PIN_SENSOR_MOSI_0;
  
  sensorPins[i++]=PIN_SENSOR_CSEL_1;
  sensorPins[i++]=PIN_SENSOR_CLK_1;
  sensorPins[i++]=PIN_SENSOR_MISO_1;
  sensorPins[i++]=PIN_SENSOR_MOSI_1;
  
  sensorPins[i++]=PIN_SENSOR_CSEL_2;
  sensorPins[i++]=PIN_SENSOR_CLK_2;
  sensorPins[i++]=PIN_SENSOR_MISO_2;
  sensorPins[i++]=PIN_SENSOR_MOSI_2;
  
  sensorPins[i++]=PIN_SENSOR_CSEL_3;
  sensorPins[i++]=PIN_SENSOR_CLK_3;
  sensorPins[i++]=PIN_SENSOR_MISO_3;
  sensorPins[i++]=PIN_SENSOR_MOSI_3;
  
  sensorPins[i++]=PIN_SENSOR_CSEL_4;
  sensorPins[i++]=PIN_SENSOR_CLK_4;
  sensorPins[i++]=PIN_SENSOR_MISO_4;
  sensorPins[i++]=PIN_SENSOR_MOSI_4;
  
  sensorPins[i++]=PIN_SENSOR_CSEL_5;
  sensorPins[i++]=PIN_SENSOR_CLK_5;
  sensorPins[i++]=PIN_SENSOR_MISO_5;
  sensorPins[i++]=PIN_SENSOR_MOSI_5;

  for(i=0;i<NUM_SENSORS;++i) {
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
  for(int i=0;i<NUM_SENSORS;++i) {
    if(getSensorRawValue(i,rawValue)) continue;
    v = extractAngleFromRawValue(rawValue);
    // Some of these are negative because the sensor is reading the opposite rotation from the Robot Overlord simulation.
    // Robot Overlord has the final say, so these are flipped to match the simulation.
    // This is the only place motor direction should ever be inverted.
    if(i!=1 && i!=2) v=-v;
    v -= axies[i].homePos;
    while(v<-180) v+=360;
    while(v> 180) v-=360;
    sensorAngles[i] = v;
  }
}

#else // MACHINE_STYLE == SIXI

void reportAllAngleValues() {}

#endif

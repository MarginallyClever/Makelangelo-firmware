//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#include "configure.h"
#include "motor.h"
#include "sdcard.h"
#include "lcd.h"

#include <SPI.h>  // pkm fix for Arduino 1.5

#include "Vector3.h"


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------

// robot UID
int robot_uid = 0;

// description of the machine's position
Axis axies[NUM_AXIES];

// length of belt when weights hit limit switch
float calibrateRight  = 1011.0;
float calibrateLeft   = 1011.0;

// plotter position.
float feed_rate = DEFAULT_FEEDRATE;
float acceleration = DEFAULT_ACCELERATION;
float step_delay;


#if MACHINE_STYLE == SIXI
uint32_t reportDelay = 0;
#endif


#ifdef HAS_WIFI

#ifdef ESP32
#include <WiFi.h>
#endif

#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#endif

const char* SSID_NAME = WIFI_SSID_NAME;
const char* SSID_PASS = WIFI_SSID_PASS;
WiFiUDP port;
unsigned int localPort = 9999;

#endif  // HAS_WIFI




//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------




void findStepDelay() {
  step_delay = 1000000.0f / (DEFAULT_FEEDRATE / MM_PER_STEP);
}


// returns angle of dy/dx as a value from 0...2PI
float atan3(float dy, float dx) {
  float a = atan2(dy, dx);
  if (a < 0) a += (PI * 2.0);
  return a;
}


/**
   feed rate is given in units/min and converted to cm/s
*/
void setFeedRate(float v1) {
  if ( feed_rate != v1 ) {
    feed_rate = v1;
    if (feed_rate < MIN_FEEDRATE) feed_rate = MIN_FEEDRATE;
#ifdef VERBOSE
    Serial.print(F("F="));
    Serial.println(feed_rate);
#endif
  }
}


/**
   @param delay in microseconds
*/
void pause(const long us) {
  delay(us / 1000);
  delayMicroseconds(us % 1000);
}


/**
   Test that IK(FK(A))=A
*/
void testKinematics() {
  long A[NUM_MOTORS], i, j;
  float axies1[NUM_AXIES];
  float axies2[NUM_AXIES];

  for (i = 0; i < 3000; ++i) {
    for (j = 0; j < NUM_AXIES; ++j) {
      axies1[j] = random(axies[j].limitMin, axies[j].limitMax);
    }

    IK(axies1, A);
    FK(A, axies2);

    for (j = 0; j < NUM_AXIES; ++j) {
      Serial.print('\t');
      Serial.print(AxisNames[j]);
      Serial.print(axies1[j]);
    }
    for (j = 0; j < NUM_MOTORS; ++j) {
      Serial.print('\t');
      Serial.print(MotorNames[j]);
      Serial.print(A[j]);
    }
    for (j = 0; j < NUM_AXIES; ++j) {
      Serial.print('\t');
      Serial.print(AxisNames[j]);
      Serial.print('\'');
      Serial.print(axies2[j]);
    }
    for (j = 0; j < NUM_AXIES; ++j) {
      Serial.print(F("\td"));
      Serial.print(AxisNames[j]);
      Serial.print('=');
      Serial.print(axies2[j] - axies1[j]);
    }
    Serial.println();
  }
}


/**
   Split long moves into sub-moves if needed.
   @input pos NUM_AXIES floats describing destination coordinates
   @input new_feed_rate speed to travel along arc
*/
void lineSafe(float *pos, float new_feed_rate_mms) {
  // Remember the feed rate.  This value will be used whenever no feedrate is given in a command, so it MUST be saved BEFORE the dial adjustment.
  // otherwise the feedrate will slowly fall or climb as new commands are processed.
  feed_rate = new_feed_rate_mms;

#ifdef HAS_LCD
  // use LCD to adjust speed while drawing
  new_feed_rate_mms *= (float)speed_adjust * 0.01f;
#endif

  // split up long lines to make them straighter
  float delta[NUM_AXIES];
  float startPos[NUM_AXIES];
  float lenSquared = 0;

  for(ALL_AXIES(i)) {
    startPos[i] = axies[i].pos;
    delta[i] = pos[i] - startPos[i];
    lenSquared += sq(delta[i]);
  }

#if MACHINE_STYLE == POLARGRAPH
  if(delta[0]==0 && delta[1]==0) {
    // only moving Z, don't split the line.
    motor_line(pos, new_feed_rate_mms, abs(delta[2]));
    return;
  }
#endif

  float len_mm = sqrt(lenSquared);
  if(abs(len_mm)<0.000001f) return;

  const float seconds = len_mm / new_feed_rate_mms;
  uint16_t segments = seconds * SEGMENTS_PER_SECOND;
  if(segments<1) segments=1;
  
#ifdef HAS_GRIPPER
  // if we have a gripper and only gripper is moving, don't split the movement.
  if(lenSquared == sq(delta[6])) {
    Serial.println("only t");
    segments=1;
    Serial.print("seconds=");  Serial.println(seconds);
    Serial.print("len_mm=");  Serial.println(len_mm);
    Serial.print("new_feed_rate_mms=");  Serial.println(new_feed_rate_mms);
  }
#endif
  
  const float inv_segments = 1.0f / float(segments);
  const float segment_len_mm = len_mm * inv_segments;
  
  for(ALL_AXIES(i)) delta[i] *= inv_segments;
  
  while(--segments) {
    for(ALL_AXIES(i)) startPos[i] += delta[i];
    
    motor_line(startPos, new_feed_rate_mms,segment_len_mm);
  }

  // guarantee we stop exactly at the destination (no rounding errors).
  motor_line(pos, new_feed_rate_mms,segment_len_mm);

//  Serial.print("P");  Serial.println(movesPlanned());
}


/**
   This method assumes the limits have already been checked.
   This method assumes the start and end radius match.
   This method assumes arcs are not >180 degrees (PI radians)
   @input cx center of circle x value
   @input cy center of circle y value
   @input destination point where movement ends
   @input dir - ARC_CW or ARC_CCW to control direction of arc
   @input new_feed_rate speed to travel along arc
*/
void arc(float cx, float cy, float *destination, char clockwise, float new_feed_rate) {
  // get radius
  float dx = axies[0].pos - cx;
  float dy = axies[1].pos - cy;
  float sr = sqrt(dx * dx + dy * dy);

  // find angle of arc (sweep)
  float sa = atan3(dy, dx);
  float ea = atan3(destination[1] - cy, destination[0] - cx);
  float er = sqrt(dx * dx + dy * dy);

  float da = ea - sa;
  if (clockwise == ARC_CW && da < 0) ea += 2 * PI;
  else if (clockwise == ARC_CCW && da > 0) sa += 2 * PI;
  da = ea - sa;
  float dr = er - sr;

  // get length of arc
  // float circ=PI*2.0*radius;
  // float len=theta*circ/(PI*2.0);
  // simplifies to
  float len1 = abs(da) * sr;
  float len = sqrt( len1 * len1 + dr * dr ); // mm

  int i, segments = ceil( len );

  float n[NUM_AXIES], scale;
  float a, r;
#if NUM_AXIES>2
  float sz = axies[2].pos;
  float z = destination[2];
#endif

  for (i = 0; i <= segments; ++i) {
    // interpolate around the arc
    scale = ((float)i) / ((float)segments);

    a = ( da * scale ) + sa;
    r = ( dr * scale ) + sr;

    n[0] = cx + cos(a) * r;
    n[1] = cy + sin(a) * r;
#if NUM_AXIES>2
    n[2] = ( z - sz ) * scale + sz;
#endif
    // send it to the planner
    lineSafe(n, new_feed_rate);
  }
}


/**
 * Instantly move the virtual plotter position.  Does not check if the move is valid.
 */
void teleport(float *pos) {
  wait_for_empty_segment_buffer();

  //Serial.println(F("Teleport"));
  for(ALL_AXIES(i)) {
    axies[i].pos = pos[i];
    //Serial.println(pos[i]);
  }

  long steps[NUM_MUSCLES];
  IK(pos, steps);
  motor_set_step_count(steps);
}


void setHome(float *pos) {
  int i;
  for (i = 0; i < NUM_AXIES; ++i) {
    axies[i].homePos = pos[i];
  }
  eepromManager.saveHome();
}


void meanwhile() {
#if MACHINE_STYLE == SIXI
  //Serial.println(REPORT_ANGLES_CONTINUOUSLY?"Y":"N");
  
  sensorManager.updateAll();

  if( REPORT_ANGLES_CONTINUOUSLY ) {
    if( millis() > reportDelay ) {
      reportDelay = millis() + 100;
      parser.D17();
    }
  }

#if defined(HAS_GRIPPER)
  gripper.update();
#endif

#endif  // MACHINE_STYLE == SIXI

#ifdef DEBUG_STEPPING
  debug_stepping();
#endif  // DEBUG_STEPPING
}


void unitTestWrapDegrees() {
  // unit test WRAP_DEGREES
  for(float i=-360;i<=360;i+=0.7) {
    Serial.print(i);
    Serial.print("\t");
    Serial.println(WRAP_DEGREES(i));
  }
}

void unitTestBitMacros() {
  uint32_t a=0;
  Serial.print("on=");
  SET_BIT_ON(a,1);
  Serial.println(a,BIN);
  
  Serial.print("test=");
  Serial.println(TEST(a,1)?"on":"off");

  Serial.print("off=");
  SET_BIT_OFF(a,1);
  Serial.println(a,BIN);

  Serial.print("test=");
  Serial.println(TEST(a,1)?"on":"off");

  Serial.print("flip=");
  FLIP_BIT(a,1);
  Serial.println(a,BIN);

  Serial.print("test=");
  Serial.println(TEST(a,1)?"on":"off");

  Serial.print("set=");
  SET_BIT(a,1,false);
  Serial.println(a,BIN);
  
  Serial.print("test=");
  Serial.println(TEST(a,1)?"on":"off");

  while(1) {}
}


void reportAllMotors() {
  for(ALL_MOTORS(i)) {
    Serial.println(motors[i].letter);
    Serial.print("\tangleHome=");        Serial.println(axies[i].homePos);
#if MACHINE_STYLE == SIXI
    Serial.print("\tsensor=");           Serial.println(sensorManager.sensors[i].angle);
#endif
  }
  Serial.println();
}


// runs once on machine start
void setup() {
  parser.start();
    
  eepromManager.loadAll();

  //unitTestWrapDegrees();
  //unitTestBitMacros();
  
#ifdef HAS_SD
  SD_setup();
#endif
#ifdef HAS_LCD
  LCD_setup();
#endif

  //clockISRProfile();

  motor_setup();
  findStepDelay();

  //easyPWM_init();

  // initialize the plotter position.
  float pos[NUM_AXIES];
  for(ALL_AXIES(i)) pos[i] = 0;
  
#ifdef MACHINE_HAS_LIFTABLE_PEN
  if (NUM_AXIES >= 3) pos[2] = PEN_UP_ANGLE;
#endif

#if MACHINE_STYLE==SIXI && defined(HAS_GRIPPER)
  gripper.setup();
#endif

  teleport(pos);

  setFeedRate(DEFAULT_FEEDRATE);

  robot_setup();

  //reportAllMotors();

  parser.M100();
  parser.ready();
}


// after setup runs over and over.
void loop() {
  parser.update();
  
#ifdef HAS_SD
  SD_check();
#endif
#ifdef HAS_LCD
  LCD_update();
#endif

  // The PC will wait forever for the ready signal.
  // if Arduino hasn't received a new instruction in a while, send ready() again
  // just in case USB garbled ready and each half is waiting on the other.
  if ( !segment_buffer_full() && (millis() - parser.lastCmdTimeMs ) > TIMEOUT_OK ) {
#ifdef HAS_TMC2130
    {
      uint32_t drv_status = driver_0.DRV_STATUS();
      uint32_t stallValue = (drv_status & SG_RESULT_bm) >> SG_RESULT_bp;
      Serial.print(stallValue, DEC);
      Serial.print('\t');
    }
    {
      uint32_t drv_status = driver_1.DRV_STATUS();
      uint32_t stallValue = (drv_status & SG_RESULT_bm) >> SG_RESULT_bp;
      Serial.println(stallValue, DEC);
    }
#endif
    parser.ready();
  }

  meanwhile();
}

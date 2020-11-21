# 1 "C:\\Users\\profs\\AppData\\Local\\Temp\\tmp1aknkc6k"
#include <Arduino.h>
# 1 "D:/GIT/Makelangelo-firmware-pio/Makelangelo-firmware/Makelangelo-firmware.ino"
# 11 "D:/GIT/Makelangelo-firmware-pio/Makelangelo-firmware/Makelangelo-firmware.ino"
#include "configure.h"
#include "motor.h"
#include "sdcard.h"
#include "lcd.h"
#include <SPI.h>
#include "Vector3.h"

#ifdef PIO



  #define XSTR(x) #x
  #define STR(x) XSTR(x)
#endif






int robot_uid = 0;


Axis axies[NUM_AXIES];


float calibrateRight = 1011.0;
float calibrateLeft = 1011.0;


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

#endif
# 79 "D:/GIT/Makelangelo-firmware-pio/Makelangelo-firmware/Makelangelo-firmware.ino"
void findStepDelay();
float atan3(float dy, float dx);
void setFeedRate(float v1);
void pause(const long us);
void testKinematics();
void lineSafe(float *pos, float new_feed_rate_mms);
void arc(float cx, float cy, float *destination, char clockwise, float new_feed_rate);
void teleport(float *pos);
void setHome(float *pos);
void meanwhile();
void unitTestWrapDegrees();
void unitTestBitMacros();
void reportAllMotors();
void setup();
void loop();
#line 79 "D:/GIT/Makelangelo-firmware-pio/Makelangelo-firmware/Makelangelo-firmware.ino"
void findStepDelay() {
  step_delay = 1000000.0f / (DEFAULT_FEEDRATE / MM_PER_STEP);
}



float atan3(float dy, float dx) {
  float a = atan2(dy, dx);
  if (a < 0) a += (PI * 2.0);
  return a;
}





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





void pause(const long us) {
  delay(us / 1000);
  delayMicroseconds(us % 1000);
}





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







void lineSafe(float *pos, float new_feed_rate_mms) {


  feed_rate = new_feed_rate_mms;

#ifdef HAS_LCD

  new_feed_rate_mms *= (float)speed_adjust * 0.01f;
#endif


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

  if(lenSquared == sq(delta[6])) {
    Serial.println("only t");
    segments=1;
    Serial.print("seconds="); Serial.println(seconds);
    Serial.print("len_mm="); Serial.println(len_mm);
    Serial.print("new_feed_rate_mms="); Serial.println(new_feed_rate_mms);
  }
#endif

  const float inv_segments = 1.0f / float(segments);
  const float segment_len_mm = len_mm * inv_segments;

  for(ALL_AXIES(i)) delta[i] *= inv_segments;

  while(--segments) {
    for(ALL_AXIES(i)) startPos[i] += delta[i];

    motor_line(startPos, new_feed_rate_mms,segment_len_mm);
  }


  motor_line(pos, new_feed_rate_mms,segment_len_mm);


}
# 239 "D:/GIT/Makelangelo-firmware-pio/Makelangelo-firmware/Makelangelo-firmware.ino"
void arc(float cx, float cy, float *destination, char clockwise, float new_feed_rate) {

  float dx = axies[0].pos - cx;
  float dy = axies[1].pos - cy;
  float sr = sqrt(dx * dx + dy * dy);


  float sa = atan3(dy, dx);
  float ea = atan3(destination[1] - cy, destination[0] - cx);
  float er = sqrt(dx * dx + dy * dy);

  float da = ea - sa;
  if (clockwise == ARC_CW && da < 0) ea += 2 * PI;
  else if (clockwise == ARC_CCW && da > 0) sa += 2 * PI;
  da = ea - sa;
  float dr = er - sr;





  float len1 = abs(da) * sr;
  float len = sqrt( len1 * len1 + dr * dr );

  int i, segments = ceil( len );

  float n[NUM_AXIES], scale;
  float a, r;
#if NUM_AXIES>2
  float sz = axies[2].pos;
  float z = destination[2];
#endif

  for (i = 0; i <= segments; ++i) {

    scale = ((float)i) / ((float)segments);

    a = ( da * scale ) + sa;
    r = ( dr * scale ) + sr;

    n[0] = cx + cos(a) * r;
    n[1] = cy + sin(a) * r;
#if NUM_AXIES>2
    n[2] = ( z - sz ) * scale + sz;
#endif

    lineSafe(n, new_feed_rate);
  }
}





void teleport(float *pos) {
  wait_for_empty_segment_buffer();


  for(ALL_AXIES(i)) {
    axies[i].pos = pos[i];

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

#endif

#ifdef DEBUG_STEPPING
  debug_stepping();
#endif
}


void unitTestWrapDegrees() {

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
    Serial.print("\tangleHome="); Serial.println(axies[i].homePos);
#if MACHINE_STYLE == SIXI
    Serial.print("\tsensor="); Serial.println(sensorManager.sensors[i].angle);
#endif
  }
  Serial.println();
}



void setup() {
  parser.start();

  eepromManager.loadAll();




#ifdef HAS_SD
  SD_setup();
#endif
#ifdef HAS_LCD
  LCD_setup();
#endif



  motor_setup();
  findStepDelay();




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



  parser.M100();
  parser.ready();
}



void loop() {
  parser.update();

#ifdef HAS_SD
  SD_check();
#endif
#ifdef HAS_LCD
  LCD_update();
#endif




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
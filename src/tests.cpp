#include "configure.h"
#include "motor.h"


void tmc2130_test1() {
  //tmc2130_ms(2);
  //tmc2130_status();

  digitalWrite(MOTOR_0_ENABLE_PIN,LOW);
  digitalWrite(MOTOR_0_DIR_PIN,LOW);
  for(int i=0;i<1000;++i) {
    digitalWrite(MOTOR_0_STEP_PIN,HIGH);
    digitalWrite(MOTOR_0_STEP_PIN,LOW);
    delay(1);
  }
  digitalWrite(MOTOR_0_DIR_PIN,HIGH);
  for(int i=0;i<1000;++i) {
    digitalWrite(MOTOR_0_STEP_PIN,HIGH);
    digitalWrite(MOTOR_0_STEP_PIN,LOW);
    delay(1);
  }
}

void unitTestWrapDegrees() {
  // unit test WRAP_DEGREES
  for (float i = -360; i <= 360; i += 0.7) {
    Serial.print(i);
    Serial.print("\t");
    Serial.println(WRAP_DEGREES(i));
  }
}

void unitTestBitMacros() {
  uint32_t a = 0;
  Serial.print("on=");
  SET_BIT_ON(a, 1);
  Serial.println(a, BIN);

  Serial.print("test=");
  Serial.println(TEST(a, 1) ? "on" : "off");

  Serial.print("off=");
  SET_BIT_OFF(a, 1);
  Serial.println(a, BIN);

  Serial.print("test=");
  Serial.println(TEST(a, 1) ? "on" : "off");

  Serial.print("flip=");
  FLIP_BIT(a, 1);
  Serial.println(a, BIN);

  Serial.print("test=");
  Serial.println(TEST(a, 1) ? "on" : "off");

  Serial.print("set=");
  SET_BIT(a, 1, false);
  Serial.println(a, BIN);

  Serial.print("test=");
  Serial.println(TEST(a, 1) ? "on" : "off");

  while (1) {}
}

// Test that IK(FK(A))=A
void testKinematics() {
  long A[NUM_MOTORS], i, j;
  float axies1[NUM_AXIES];
  float axies2[NUM_AXIES];

  for (i = 0; i < 3000; ++i) {
    for (j = 0; j < NUM_AXIES; ++j) { axies1[j] = random(axies[j].limitMin, axies[j].limitMax); }

    IK(axies1, A);
    FK(A, axies2);

    for (j = 0; j < NUM_AXIES; ++j) {
      Serial.print('\t');
      Serial.print(AxisNames[j]);
      Serial.print(axies1[j]);
    }
    for (j = 0; j < NUM_MOTORS; ++j) {
      Serial.print('\t');
      Serial.print(motors[j].letter);
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

void testCircle() {
  robot_findHome();
  float p[NUM_AXIES];
  p[2]=0;

  float r = 100;
  while(true) {
    for(float i=0;i<360;i++) {
      p[0]=r*cos(i*PI/180.0);
      p[1]=r*sin(i*PI/180.0);
      planner.bufferLine(p,desiredFeedRate);
    }
  }
}
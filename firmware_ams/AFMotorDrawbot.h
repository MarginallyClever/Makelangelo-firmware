#ifndef _AFMotorDrawbot_h_
#define _AFMotorDrawbot_h_
//------------------------------------------------------------------------------
// Adafruit Motor shield library, modified for Drawbot
// copyright Dan Royer, 2012
// this code is public domain, enjoy!
//------------------------------------------------------------------------------

#include <inttypes.h>
#include <Wire.h>

//------------------------------------------------------------------------------

//#define MOTORDEBUG 1

#define MICROSTEPS 2  // 8 or 16

// Bit positions in the 74HCT595 shift register output
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR4_A 0
#define MOTOR4_B 6
#define MOTOR3_A 5
#define MOTOR3_B 7

// Constants that the user passes in to the motor calls
#define FORWARD 1
#define BACKWARD -1
#define BRAKE 0
#define RELEASE 2

// Constants that the user passes in to the stepper calls
#define SINGLE 1
#define DOUBLE 2
#define INTERLEAVE 3
#define MICROSTEP 4

// Arduino pin names for interface to 74HCT595 latch
#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8

//------------------------------------------------------------------------------

class AFMotorController {
public:
  AFMotorController();
  void enable();
  friend class AF_DCMotor;
  void latch_tx();
};


class AF_Stepper {
 public:
  AF_Stepper(uint16_t, uint8_t);
  void step(uint16_t steps, uint8_t dir);
  void setSpeed(uint16_t);
  uint8_t onestep(uint8_t dir);
  void release();
  uint16_t revsteps; // # steps per revolution
  uint8_t steppernum;
  uint32_t usperstep;
 private:
  uint8_t currentstep;
  uint8_t a, b, c, d;
};


//------------------------------------------------------------------------------
#endif

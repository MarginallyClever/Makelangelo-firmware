//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#include "configure.h"
#include "sdcard.h"
#include "lcd.h"

#include <SPI.h>  // pkm fix for Arduino 1.5

#include "vector3.h"

//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------

// robot UID
int robot_uid = 0;

// description of the machine's position
Axis axies[NUM_AXIES];

// length of belt when weights hit limit switch
float calibrateRight = 1011.0;
float calibrateLeft  = 1011.0;

// plotter position.
float desiredFeedRate     = DEFAULT_FEEDRATE;
float desiredAcceleration = DEFAULT_ACCELERATION;

#if MACHINE_STYLE == SIXI
uint32_t reportDelay = 0;
#endif

#ifdef HAS_WIFI

#  ifdef ESP32
#    include <WiFi.h>
#  endif

#  ifdef ESP8266
#    include <ESP8266WiFi.h>
#    include <WiFiUdp.h>
#  endif

const char *SSID_NAME = WIFI_SSID_NAME;
const char *SSID_PASS = WIFI_SSID_PASS;
WiFiUDP port;
unsigned int localPort = 9999;

#endif  // HAS_WIFI

//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------

/**
 * calculate microseconds-per-step.
 * step_per_units = 1 / UNITS_PER_STEP
 * steps_per_second = step_per_units * desiredFeedRate (units/s)
 * microseconds_per_step = 1M microseconds / steps_per_second
 **/
hal_timer_t findStepDelay() {
  float stepsPerSecond = desiredFeedRate * STEPS_PER_UNIT;
  hal_timer_t delay = (hal_timer_t)(1000000.0f / stepsPerSecond);
  return delay;
}

void setFeedRate(float units_per_s) {
  if (desiredFeedRate != units_per_s) {
    desiredFeedRate = _MAX(_MIN(units_per_s,MAX_FEEDRATE),MIN_FEEDRATE);
  }
}

void pause(const uint32_t us) {
  delay(us / 1000);
  delayMicroseconds(us % 1000);
}

/**
 * Instantly move the virtual plotter position.  Does not check if the move is valid.
 */
void teleport(float *pos) {
  planner.wait_for_empty_segment_buffer();

  // MYSERIAL1.println(F("Teleport"));
  for (ALL_AXIES(i)) {
    axies[i].pos = pos[i];
    // MYSERIAL1.println(pos[i]);
  }

  int32_t steps[NUM_MUSCLES];
  IK(pos, steps);
  motor.set_step_count(steps);
}

void setHome(float *pos) {
  uint8_t i;
  uint8_t j=0;

  for (i = 0; i < NUM_AXIES; ++i) {
    if(axies[i].homePos != pos[i]) {
      axies[i].homePos = pos[i];
      j=1;
    }
  }

  if(j==1) {
    eepromManager.saveHome();
  }
}

void meanwhile() {
#if MACHINE_STYLE == SIXI
  // SERIAL_ECHO(REPORT_ANGLES_CONTINUOUSLY?"Y":"N");

  sensorManager.updateAll();

  if (REPORT_ANGLES_CONTINUOUSLY) {
    if (millis() > reportDelay) {
      reportDelay = millis() + 100;
      parser.D17();
    }
  }

#  if defined(HAS_GRIPPER)
  gripper.update();
#  endif
#endif  // MACHINE_STYLE == SIXI

#ifdef DEBUG_STEPPING
  Stepper::isr();
#endif  // DEBUG_STEPPING
}

void reportAllMotors() {
  for (ALL_MOTORS(i)) {
    MYSERIAL1.println(motors[i].letter);
    MYSERIAL1.print("\tangleHome=");
    MYSERIAL1.println(axies[i].homePos);
#if MACHINE_STYLE == SIXI
    MYSERIAL1.print("\tsensor=");
    MYSERIAL1.println(sensorManager.sensors[i].angle);
#endif
  }
  SERIAL_EOL();
}


void reportSpeeds() {
  // The stepping frequency limits for each multistepping rate
  static const uint32_t limit[] PROGMEM = {
    (  MAX_STEP_ISR_FREQUENCY_1X     ),
    (  MAX_STEP_ISR_FREQUENCY_2X >> 1),
    (  MAX_STEP_ISR_FREQUENCY_4X >> 2),
    (  MAX_STEP_ISR_FREQUENCY_8X >> 3),
    ( MAX_STEP_ISR_FREQUENCY_16X >> 4),
    ( MAX_STEP_ISR_FREQUENCY_32X >> 5),
    ( MAX_STEP_ISR_FREQUENCY_64X >> 6),
    (MAX_STEP_ISR_FREQUENCY_128X >> 7)
  };

  for(int i=0;i<8;++i) {
    SERIAL_ECHO(i);
    SERIAL_CHAR(' ');
    SERIAL_ECHOLN(pgm_read_dword(&limit[i]));
  }

  SERIAL_ECHOLNPAIR("ISR_EXECUTION_CYCLES(1UL)=",ISR_EXECUTION_CYCLES(1UL));
  SERIAL_ECHOLNPAIR("ISR_BASE_CYCLES=",ISR_BASE_CYCLES);
  SERIAL_ECHOLNPAIR("ISR_S_CURVE_CYCLES=",ISR_S_CURVE_CYCLES);
  SERIAL_ECHOLNPAIR("ISR_LOOP_CYCLES=",ISR_LOOP_CYCLES);
  
  SERIAL_ECHOLNPAIR("ISR_LOOP_BASE_CYCLES=",ISR_LOOP_BASE_CYCLES);
  SERIAL_ECHOLNPAIR("MIN_STEPPER_PULSE_CYCLES=",MIN_STEPPER_PULSE_CYCLES);
  SERIAL_ECHOLNPAIR("MIN_ISR_LOOP_CYCLES=",MIN_ISR_LOOP_CYCLES);
}

// runs once on machine start
void setup() {
  parser.start();

  HAL_init();
  
  eepromManager.loadAll();

  // unitTestWrapDegrees();
  // unitTestBitMacros();

#ifdef HAS_SD
  sd.setup();
#endif
#ifdef HAS_LCD
  LCD_setup();
#endif

  planner.zeroSpeeds();
  motor.setup();
  // easyPWM_init();

  // initialize the plotter position.
  float pos[NUM_AXIES];
  for (ALL_AXIES(i)) pos[i] = 0;

#ifdef MACHINE_HAS_LIFTABLE_PEN
  if (NUM_AXIES >= 3) pos[2] = PEN_UP_ANGLE;
#endif

#if MACHINE_STYLE == SIXI && defined(HAS_GRIPPER)
  gripper.setup();
#endif

  teleport(pos);
  setFeedRate(DEFAULT_FEEDRATE);
  robot_setup();
  // reportAllMotors();

  parser.M100();
  parser.ready();

  // run tests
  //testCircle();
  //reportSpeeds();
}

// after setup runs over and over.
void loop() {
#ifdef HAS_SD
  sd.check();
#endif
#ifdef HAS_LCD
  LCD_update();
#endif

  parser.update();

  parser.advance();
  // The PC will wait forever for the ready signal.
  // if Arduino hasn't received a new instruction in a while, send ready() again
  // just in case USB garbled ready and each half is waiting on the other.
  if (!planner.movesPlanned() && (millis() - parser.lastCmdTimeMs) > TIMEOUT_OK) {
#ifdef HAS_TMC2130
    // for debugging limit switches
    //tmc2130_status();
#endif
    parser.ready();
  }

  meanwhile();
}
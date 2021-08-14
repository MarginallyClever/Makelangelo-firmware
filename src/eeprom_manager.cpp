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
#include "eeprom_manager.h"

#include <EEPROM.h>
#include <Arduino.h>  // for type definitions

EEPROMManager eepromManager;

// from http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1234477290/3
long EEPROMManager::readLong(int ee) {
  long value = 0;
  byte* p    = (byte*)(void*)&value;
  for (uint16_t i = 0; i < sizeof(value); i++) *p++ = EEPROM.read(ee++);
  return value;
}

// 2020-01-31 Dan added check to not update EEPROM if value is unchanged.
// from http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1234477290/3
// returns 1 if the value was changed.
uint8_t EEPROMManager::writeLong(int ee, long value) {
  if (readLong(ee) == value) return 0;

  byte* p = (byte*)(void*)&value;
  for (uint16_t i = 0; i < sizeof(value); i++) EEPROM.write(ee++, *p++);

  return 1;
}

uint8_t EEPROMManager::loadVersion() {
  return EEPROM.read(EEPROM_VERSION);
}

void EEPROMManager::saveUID() {
  SERIAL_ECHOLNPGM("Saving UID.");
  writeLong(EEPROM_UUID, (long)robot_uid);
}

uint8_t EEPROMManager::loadUID() {
  return EEPROM.read(EEPROM_VERSION);
}

void EEPROMManager::saveLimits() {
  SERIAL_ECHOLNPGM("Saving limits.");
  int j = EEPROM_LIMITS;
  for (ALL_AXIES(i)) {
    writeLong(j, axies[i].limitMax * 100);
    j += 4;
    writeLong(j, axies[i].limitMin * 100);
    j += 4;
  }
}

void EEPROMManager::loadLimits() {
  int j = EEPROM_LIMITS;
  for (ALL_AXIES(i)) {
    axies[i].limitMax = (float)readLong(j) / 100.0f;
    j += 4;
    axies[i].limitMin = (float)readLong(j) / 100.0f;
    j += 4;
    // MYSERIAL1.print("Axis ");
    // MYSERIAL1.print(i);
    // MYSERIAL1.print(" Min ");
    // MYSERIAL1.print(axies[i].limitMin);
    // MYSERIAL1.print(" Max ");
    // MYSERIAL1.print(axies[i].limitMax);
    // SERIAL_EOL();
  }
}

/**
 * @param limits NUM_AXIES*2 floats.  Each pair is one float for max limit and one for min limit.
 */
void EEPROMManager::adjustLimits(float* limits) {
  SERIAL_ECHOLNPGM("Adjusting limits.");
  int j       = 0;
  int changed = 0;
  float v;
  for (ALL_AXIES(i)) {
    // max test
    v = floorf(limits[j] * 100.0f) / 100.0f;
    if (v != axies[i].limitMax) {
      axies[i].limitMax = v;
      changed           = 1;
    }
    j++;
    // min test
    v = floorf(limits[j] * 100.0f) / 100.0f;
    if (v != axies[i].limitMin) {
      axies[i].limitMin = v;
      changed           = 1;
    }
    j++;
  }

  if (changed != 0) { saveLimits(); }
}

void EEPROMManager::saveHome() {
  SERIAL_ECHOLNPGM("Saving home.");
  int j = EEPROM_HOME;
  for (ALL_AXIES(i)) {
    writeLong(j, (long)(axies[i].homePos * 100.0f));
    j += SIZEOF_FLOAT_BYTES;
  }
}

void EEPROMManager::loadHome() {
  // MYSERIAL1.print(F("Loading home:"));
  int j = EEPROM_HOME;
  for (ALL_AXIES(i)) {
    axies[i].homePos = (float)readLong(j) / 100.0f;
    // MYSERIAL1.print(' ');
    // MYSERIAL1.print(motors[i].letter);
    // MYSERIAL1.print(axies[i].homePos);
    j += SIZEOF_FLOAT_BYTES;
  }
  // SERIAL_EOL();
}

void EEPROMManager::saveCalibration() {
  SERIAL_ECHOLNPGM("Saving calibration.");
  writeLong(EEPROM_CALIBRATION_LEFT, calibrateLeft * 100);
  writeLong(EEPROM_CALIBRATION_RIGHT, calibrateRight * 100);
}

void EEPROMManager::loadCalibration() {
  calibrateLeft  = (float)readLong(EEPROM_CALIBRATION_LEFT) / 100.0f;
  calibrateRight = (float)readLong(EEPROM_CALIBRATION_RIGHT) / 100.0f;
}

void EEPROMManager::saveSPU() {
  SERIAL_ECHOLNPGM("Saving SPU.");
  int j = EEPROM_SPU;
  for(ALL_MUSCLES(i)) {
    writeLong(j, motor_spu[i]*100.0f);
    j+=SIZEOF_FLOAT_BYTES;
  }
}

void EEPROMManager::loadSPU() {
  int j = EEPROM_SPU;
  for(ALL_MUSCLES(i)) {
    motor_spu[i]  = (float)readLong(j) / 100.0f;
    j+=SIZEOF_FLOAT_BYTES;
  }
}

void EEPROMManager::saveJerk() {
  SERIAL_ECHOLNPGM("Saving jerk.");
  int j = EEPROM_JERK;
  for(ALL_MUSCLES(i)) {
    writeLong(j, max_jerk[i]*100.0f);
    j+=SIZEOF_FLOAT_BYTES;
  }
}

void EEPROMManager::loadJerk() {
  int j = EEPROM_JERK;
  for(ALL_MUSCLES(i)) {
    max_jerk[i]  = (float)readLong(j) / 100.0f;
    j+=SIZEOF_FLOAT_BYTES;
  }
}

void EEPROMManager::saveStepRate() {
  SERIAL_ECHOLNPGM("Saving step rate.");
  int j = EEPROM_STEP_RATE;
  for(ALL_MUSCLES(i)) {
    writeLong(j, max_step_rate[i]*100.0f);
    j+=SIZEOF_FLOAT_BYTES;
  }
}

void EEPROMManager::loadStepRate() {
  int j = EEPROM_STEP_RATE;
  for(ALL_MUSCLES(i)) {
    max_step_rate[i]  = (float)readLong(j) / 100.0f;
    j+=SIZEOF_FLOAT_BYTES;
  }
}

void EEPROMManager::saveAll() {
  saveUID();
  saveLimits();
  saveHome();
  saveSPU();
  saveJerk();
  saveStepRate();
}

void EEPROMManager::loadAll() {
  char versionNumber = loadVersion();
  if (versionNumber != FIRMWARE_VERSION) {
    // If not the current FIRMWARE_VERSION or the FIRMWARE_VERSION is sullied (i.e. unknown data)
    // Update the version number
    EEPROM.write(EEPROM_VERSION, FIRMWARE_VERSION);
#if MACHINE_STYLE == POLARGRAPH
#  if MAKELANGELO_HARDWARE_VERSION == 5 || MAKELANGELO_HARDWARE_VERSION == 6
    parser.M502();
#  endif
#endif
    if(versionNumber==10) {
      saveSPU();
    }
  }

  // Retrieve stored configuration
  loadUID();
  loadLimits();
  loadHome();
  loadCalibration();
  loadSPU();
  loadJerk();
  loadStepRate();
}

void EEPROMManager::reportAll() {
  // model and UID
  parser.sayModelAndUID();
  // firmware version
  parser.D5();
  // build date & time
  parser.sayBuildDateAndTime();
  // limits
  parser.M101();
  // feedreate, acceleration, and home position
  parser.M114();
#if MACHINE_STYLE == POLARGRAPH
  // belt calibration
  parser.D8();
#endif
  parser.D6();
  parser.M92();
  parser.M203();
  parser.M205();
  
#if MACHINE_STYLE == SIXI
  // Sixi only home angle values
  SERIAL_ECHOLNPGM("Home angles ");
  for (ALL_MOTORS(i)) {
    SERIAL_CHAR(' ');
    SERIAL_CHAR(motors[i].letter);
    SERIAL_ECHO(axies[i].homePos);
  }
  SERIAL_EOL();
  // current angle values
  parser.D17();
#endif
}

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
#include "eepromManager.h"
#include <EEPROM.h>
#include <Arduino.h>  // for type definitions


EEPROMManager eepromManager;


// from http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1234477290/3
long EEPROMManager::readLong(int ee) {
  long value = 0;
  byte* p = (byte*)(void*)&value;
  for (uint16_t i = 0; i < sizeof(value); i++)
  *p++ = EEPROM.read(ee++);
  return value;
}


// 2020-01-31 Dan added check to not update EEPROM if value is unchanged.
// from http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1234477290/3
// returns true if the value was changed.
boolean EEPROMManager::writeLong(int ee, long value) {
  if(readLong(ee) == value) return false;
  
  byte* p = (byte*)(void*)&value;
  for (uint16_t i = 0; i < sizeof(value); i++)
  EEPROM.write(ee++, *p++);

  return true;
}


uint8_t EEPROMManager::loadVersion() {
  return EEPROM.read(ADDR_VERSION);
}


void EEPROMManager::saveUID() {
  Serial.println(F("Saving UID."));
  writeLong(ADDR_UUID,(long)robot_uid);
}

uint8_t EEPROMManager::loadUID() {
  return EEPROM.read(ADDR_VERSION);
}


void EEPROMManager::saveLimits() {
  Serial.println(F("Saving limits."));
  int i,j=ADDR_LIMITS;
  for(ALL_AXIES(i)) {
    writeLong(j,axies[i].limitMax*100);
    j+=4;
    writeLong(j,axies[i].limitMin*100);
    j+=4;
  }
}


void EEPROMManager::loadLimits() {
  int i,j=ADDR_LIMITS;
  for(ALL_AXIES(i)) {
    axies[i].limitMax = (float)readLong(j)/100.0f;
    j+=4;
    axies[i].limitMin = (float)readLong(j)/100.0f;
    j+=4;
    //Serial.print("Axis ");
    //Serial.print(i);
    //Serial.print(" Min ");
    //Serial.print(axies[i].limitMin);
    //Serial.print(" Max ");
    //Serial.print(axies[i].limitMax);
    //Serial.println();
  }
}


/**
 * @param limits NUM_AXIES*2 floats.  Each pair is one float for max limit and one for min limit.
 */
void EEPROMManager::adjustLimits(float *limits) {
  Serial.println(F("Adjusting limits."));
  int i,j=0;
  int changed=0;
  float v;
  for(ALL_AXIES(i)) {
    // max test
    v = floor(limits[j]*100.0f)/100.0f;
    if(v != axies[i].limitMax) {
      axies[i].limitMax = v;
      changed=1;
    }
    j++;
    // min test
    v = floor(limits[j]*100.0f)/100.0f;
    if(v != axies[i].limitMin) {
      axies[i].limitMin = v;
      changed=1;
    }
    j++;
  }

  if( changed != 0 ) {
    saveLimits();
  }
}


void EEPROMManager::saveHome() {
  Serial.println(F("Saving home."));
  int i,j=ADDR_HOME;
  for(ALL_AXIES(i)) {
    writeLong(j,(long)(axies[i].homePos*100.0f));
    j+=4;
  }
}


void EEPROMManager::loadHome() {
  //Serial.print(F("Loading home:"));
  int j=ADDR_HOME;
  for(ALL_AXIES(i)) {
    axies[i].homePos = (float)readLong(j)/100.0f;
    //Serial.print(' ');
    //Serial.print(motors[i].letter);
    //Serial.print(axies[i].homePos);
    j+=4;
  }
  //Serial.println();
}


void EEPROMManager::saveCalibration() {
  Serial.println(F("Saving calibration."));
  writeLong(ADDR_CALIBRATION_LEFT  ,calibrateLeft  *100);
  writeLong(ADDR_CALIBRATION_RIGHT ,calibrateRight *100);
}


void EEPROMManager::loadCalibration() {
  calibrateLeft   = (float)readLong(ADDR_CALIBRATION_LEFT  )/100.0f;
  calibrateRight  = (float)readLong(ADDR_CALIBRATION_RIGHT )/100.0f;
}

void EEPROMManager::saveAll() {
  saveUID();
  saveLimits();
  saveHome();
}


void EEPROMManager::loadAll() {
  char versionNumber = loadVersion();
  if( versionNumber != FIRMWARE_VERSION ) {
    // If not the current FIRMWARE_VERSION or the FIRMWARE_VERSION is sullied (i.e. unknown data)
    // Update the version number
    EEPROM.write(ADDR_VERSION,FIRMWARE_VERSION);
#if MAKELANGELO_HARDWARE_VERSION == 5 || MAKELANGELO_HARDWARE_VERSION == 6
    parser.M502();
#endif
  }
  
  // Retrieve stored configuration
  loadUID();
  loadLimits();
  loadHome();
  loadCalibration();
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
#if MACHINE_STYLE == SIXI
  // Sixi only home angle values
  Serial.print(F("Home angles "));
  for (ALL_MOTORS(i)) {
    Serial.print(' ');
    Serial.print(motors[i].letter);
    Serial.print(axies[i].homePos);
  }
  Serial.println();
  // current angle values
  parser.D17();
#endif
}

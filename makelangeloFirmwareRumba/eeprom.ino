//------------------------------------------------------------------------------
// Makelangelo - a mural drawing robot
// dan@marginallycelver.com 2013-12-26
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/Makelangelo for more information.
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
// Saving config
#include <EEPROM.h>
#include <Arduino.h>  // for type definitions


//------------------------------------------------------------------------------
// from http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1234477290/3
void EEPROM_writeLong(int ee, long value) {
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
  EEPROM.write(ee++, *p++);
}


//------------------------------------------------------------------------------
// from http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1234477290/3
float EEPROM_readLong(int ee) {
  long value = 0;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
  *p++ = EEPROM.read(ee++);
  return value;
}


//------------------------------------------------------------------------------
char loadVersion() {
  return EEPROM.read(ADDR_VERSION);
}


//------------------------------------------------------------------------------
void loadConfig() {
  char versionNumber = loadVersion();
  if( versionNumber != EEPROM_VERSION ) {
    // If not the current EEPROM_VERSION or the EEPROM_VERSION is sullied (i.e. unknown data)
    // Update the version number
    EEPROM.write(ADDR_VERSION,EEPROM_VERSION);
#if MAKELANGELO_HARDWARE_VERSION == 5
    adjustDimensions(50,-50,-32.5,32.5);
    adjustInversions(1,-1,1,-1);
    adjustPulleyDiameter(4.0/PI);
    savePulleyDiameter();
    saveCalibration();
#endif
  }
  
  // Retrieve stored configuration
  robot_uid=EEPROM_readLong(ADDR_UUID);
  adjustPulleyDiameter((float)EEPROM_readLong(ADDR_PULLEY_DIA1)/10000.0f);   //4 decimal places of percision is enough
  loadDimensions();
  loadInversions();
  loadHome();
  loadCalibration();
}


//------------------------------------------------------------------------------
void saveUID() {
  Serial.println(F("Saving UID."));
  EEPROM_writeLong(ADDR_UUID,(long)robot_uid);
}


//------------------------------------------------------------------------------
void savePulleyDiameter() {
  EEPROM_writeLong(ADDR_PULLEY_DIA1,pulleyDiameter*10000);
  //EEPROM_writeLong(ADDR_PULLEY_DIA2,pulleyDiameter*10000);
}


//------------------------------------------------------------------------------
void saveDimensions() {
  Serial.println(F("Saving dimensions."));
  EEPROM_writeLong(ADDR_LEFT  ,limit_xmin  *100);
  EEPROM_writeLong(ADDR_RIGHT ,limit_xmax *100);
  EEPROM_writeLong(ADDR_TOP   ,limit_ymax   *100);
  EEPROM_writeLong(ADDR_BOTTOM,limit_ymin*100);
}


//------------------------------------------------------------------------------
void loadDimensions() {
  limit_xmin = (float)EEPROM_readLong(ADDR_LEFT  )/100.0f;
  limit_xmax = (float)EEPROM_readLong(ADDR_RIGHT )/100.0f;
  limit_ymax = (float)EEPROM_readLong(ADDR_TOP   )/100.0f;
  limit_ymin = (float)EEPROM_readLong(ADDR_BOTTOM)/100.0f;
}


//------------------------------------------------------------------------------
void adjustDimensions(float newT,float newB,float newR,float newL) {
  // round off
  newT = floor(newT*100)/100.0f;
  newB = floor(newB*100)/100.0f;
  newR = floor(newR*100)/100.0f;
  newL = floor(newL*100)/100.0f;

  if( limit_ymax != newT ||
      limit_ymin != newB ||
      limit_xmax != newR ||
      limit_xmin != newL) {
        limit_ymax=newT;
        limit_ymin=newB;
        limit_xmax=newR;
        limit_xmin=newL;
        saveDimensions();
      }
}


//------------------------------------------------------------------------------
void saveInversions() {
  Serial.println(F("Saving inversions."));
  EEPROM.write(ADDR_INVL,m1i>0?1:0);
  EEPROM.write(ADDR_INVR,m2i>0?1:0);
  EEPROM.write(ADDR_INVU,m4i>0?1:0);
  EEPROM.write(ADDR_INVV,m5i>0?1:0);
}


//------------------------------------------------------------------------------
void loadInversions() {
  //Serial.println(F("Loading inversions."));
  m1i = EEPROM.read(ADDR_INVL)>0?1:-1;
  m2i = EEPROM.read(ADDR_INVR)>0?1:-1;
  m4i = EEPROM.read(ADDR_INVU)>0?1:-1;
  m5i = EEPROM.read(ADDR_INVV)>0?1:-1;
  adjustInversions(m1i,m2i,m4i,m5i);
}


//------------------------------------------------------------------------------
void saveHome() {
  Serial.println(F("Saving home."));
  EEPROM_writeLong(ADDR_HOMEX,homeX*100);
  EEPROM_writeLong(ADDR_HOMEY,homeY*100);
}


//------------------------------------------------------------------------------
void loadHome() {
  homeX = (float)EEPROM_readLong(ADDR_HOMEX)/100.0f;
  homeY = (float)EEPROM_readLong(ADDR_HOMEY)/100.0f;
}


//------------------------------------------------------------------------------
void saveCalibration() {
  Serial.println(F("Saving calibration."));
  EEPROM_writeLong(ADDR_CALIBRATION_LEFT  ,calibrateLeft  *100);
  EEPROM_writeLong(ADDR_CALIBRATION_RIGHT ,calibrateRight *100);
  EEPROM_writeLong(ADDR_CALIBRATION_BLEFT ,calibrateBLeft *100);
  EEPROM_writeLong(ADDR_CALIBRATION_BRIGHT,calibrateBRight*100);
}


//------------------------------------------------------------------------------
void loadCalibration() {
  calibrateLeft   = (float)EEPROM_readLong(ADDR_CALIBRATION_LEFT  )/100.0f;
  calibrateRight  = (float)EEPROM_readLong(ADDR_CALIBRATION_RIGHT )/100.0f;
  calibrateBLeft  = (float)EEPROM_readLong(ADDR_CALIBRATION_BLEFT )/100.0f;
  calibrateBRight = (float)EEPROM_readLong(ADDR_CALIBRATION_BRIGHT)/100.0f;
}

/**
 * This file is part of makelangelo-firmware.
 *
 * makelangelo-firmware is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * makelangelo-firmware is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with makelangelo-firmware.  If not, see <http://www.gnu.org/licenses/>.
 */

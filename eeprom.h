#ifndef EEPROM_H
#define EEPROM_H
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------


#define FIRMWARE_VERSION        9    // Increment when adding new variables
#define ADDR_VERSION            0                          // 0..255 (1 byte)
#define ADDR_UUID               (ADDR_VERSION+1)           // long - 4 bytes
#define ADDR_LIMITS             (ADDR_UUID+4)              // float - 4 bytes
#define ADDR_HOME               (ADDR_LIMITS+4*NUM_AXIES)  // float - 4 bytes * NUM_AXIES
#define ADDR_CALIBRATION_LEFT   (ADDR_HOME+4*NUM_AXIES)    // float - 4 bytes * NUM_AXIES
#define ADDR_CALIBRATION_RIGHT  (ADDR_CALIBRATION_LEFT+4)  // float - 4 bytes



/**
 * 
 */
char loadVersion();

/**
 * 
 */
void saveUID();

/**
 * 
 */
void saveLimits();

/**
 * 
 */
void loadLimits();

/**
 * @param limits NUM_AXIES pairs of floats.  each pair is one float for max limit and one for min limit.
 */
void adjustLimits(float *limits);

/**
 * 
 */
void saveHome();

/**
 * 
 */
void loadHome();

/**
 *
 */
void saveCalibration();

/**
 * 
 */
void loadCalibration();

/**
 * 
 */
void loadConfig();

#endif // EEPROM_H

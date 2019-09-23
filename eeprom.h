#ifndef EEPROM_H
#define EEPROM_H
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------


#define FIRMWARE_VERSION        10    // Increment when adding new variables
#define SIZEOF_FLOAT_BYTES      (4)
#define SIZEOF_LONG_BYTES       (4)
#define ADDR_VERSION            0                          // 0..255 (1 byte)
#define ADDR_UUID               (ADDR_VERSION+1)
#define EEPROM_UUID_LENGTH      (SIZEOF_LONG_BYTES)
#define ADDR_LIMITS             (ADDR_UUID+EEPROM_UUID_LENGTH)
#define EEPROM_LIMITS_LENGTH    (2*NUM_AXIES*SIZEOF_FLOAT_BYTES)
#define ADDR_HOME               (ADDR_LIMITS+EEPROM_LIMITS_LENGTH)
#define EEPROM_LIMITS_HOME      (NUM_AXIES*SIZEOF_FLOAT_BYTES)
#define ADDR_CALIBRATION_LEFT   (ADDR_HOME+EEPROM_LIMITS_HOME)
#define ADDR_CALIBRATION_RIGHT  (ADDR_CALIBRATION_LEFT+SIZEOF_FLOAT_BYTES)



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

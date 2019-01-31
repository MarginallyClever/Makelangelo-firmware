#ifndef EEPROM_H
#define EEPROM_H
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

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
void adjustDimensions(float *limits);

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
#pragma once
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#define FIRMWARE_VERSION          11  // Increment when adding new variables
#define SIZEOF_FLOAT_BYTES        (4)
#define SIZEOF_LONG_BYTES         (4)
#define EEPROM_VERSION            0  // 0..255 (1 byte)
#define EEPROM_UUID               (EEPROM_VERSION + 1)
#define EEPROM_UUID_LENGTH        (SIZEOF_LONG_BYTES)
#define EEPROM_LIMITS             (EEPROM_UUID + EEPROM_UUID_LENGTH)
#define EEPROM_LIMITS_LENGTH      (NUM_AXIES * 2 * SIZEOF_FLOAT_BYTES)
#define EEPROM_HOME               (EEPROM_LIMITS + EEPROM_LIMITS_LENGTH)
#define EEPROM_HOME_LENGTH        (NUM_AXIES * 1 * SIZEOF_FLOAT_BYTES)
#define EEPROM_CALIBRATION_LEFT   (EEPROM_HOME + EEPROM_HOME_LENGTH)
#define EEPROM_CALIBRATION_LENGTH (1 * SIZEOF_FLOAT_BYTES)
#define EEPROM_CALIBRATION_RIGHT  (EEPROM_CALIBRATION_LEFT + EEPROM_CALIBRATION_LENGTH)
#define EEPROM_PID                (EEPROM_CALIBRATION_RIGHT + EEPROM_CALIBRATION_LENGTH)
#define EEPROM_PID_LENGTH         (NUM_AXIES * 3 * SIZEOF_FLOAT_BYTES)
#define EEPROM_SPU                (EEPROM_PID + EEPROM_PID_LENGTH)
#define EEPROM_SPU_LENGTH         (NUM_MUSCLES * 1 * SIZEOF_FLOAT_BYTES)
#define EEPROM_JERK               (EEPROM_SPU + EEPROM_SPU_LENGTH)
#define EEPROM_JERK_LENGTH        (NUM_MUSCLES * 1 * SIZEOF_FLOAT_BYTES)
#define EEPROM_STEP_RATE          (EEPROM_JERK + EEPROM_JERK_LENGTH)
#define EEPROM_STEP_RATE_LENGTH   (NUM_MUSCLES * 1 * SIZEOF_FLOAT_BYTES)

class EEPROMManager {
 public:
  long readLong(int ee);
  uint8_t writeLong(int ee, long value);

  void saveAll();
  void loadAll();
  void reportAll();

  uint8_t loadVersion();

  void saveUID();
  uint8_t loadUID();

  void saveLimits();
  void loadLimits();

  /**
   * @param limits NUM_AXIES pairs of floats.  each pair is one float for max limit and one for min limit.
   */
  void adjustLimits(float *limits);

  void saveHome();
  void loadHome();

  void saveCalibration();
  void loadCalibration();

  //v11
  void saveSPU();
  void loadSPU();
  
  void saveJerk();
  void loadJerk();
  
  void saveStepRate();
  void loadStepRate();
};

extern EEPROMManager eepromManager;

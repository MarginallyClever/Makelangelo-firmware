#pragma once
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#ifdef HAS_SD

#  include <SPI.h>
#  include "SdFat.h"
#include "parser.h"

extern File root;

class SDCard {
public:
  static SdFat sd;
  static File root;
  static char sd_inserted;
  static char sd_printing_now;
  static char sd_printing_paused;

  static File sd_print_file;
  static float sd_percent_complete;
  static long sd_file_size;
  static long sd_bytes_read;

  static char buffer[PARSER_BUFFER_LENGTH];
  static uint8_t bufferPos;

  void load_card();

  void check();
  void setup();
  void listFiles();
  void StartPrintingFile(File toPrint);
};

extern SDCard sd;

#endif  // HAS_SD

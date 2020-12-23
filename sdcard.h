#pragma once
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#ifdef HAS_SD

#include <SPI.h>
#include "SdFat.h"


extern File root;

extern char sd_inserted;
extern char sd_printing_now;
extern char sd_printing_paused;

extern void SD_check();
extern void SD_setup();
extern void SD_listFiles();
extern void SD_StartPrintingFile(File toPrint);

#endif  // HAS_SD

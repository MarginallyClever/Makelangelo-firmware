#ifndef SDCARD_H
#define SDCARD_H
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

#ifdef HAS_SD

#include <SPI.h>
#include <SD.h>


extern File root;
extern char sd_inserted;
extern char sd_printing_now;
extern char sd_printing_paused;
extern float sd_percent_complete;
#endif

extern void SD_check();
extern void SD_setup();
extern void SD_listFiles();
extern void SD_StartPrintingFile(const char *);

#endif // SDCARD_H

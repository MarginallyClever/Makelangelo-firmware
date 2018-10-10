#ifndef SDCARD_H
#define SDCARD_H
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------


#include <SPI.h>
#include <SD.h>

#ifdef HAS_SD

extern File root;
extern char sd_inserted;
extern char sd_printing_now;
extern char sd_printing_paused;
extern float sd_percent_complete;
#endif


#endif // SDCARD_H

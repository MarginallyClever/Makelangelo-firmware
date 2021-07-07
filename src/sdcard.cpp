//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#include "configure.h"

#ifdef HAS_SD

#  include "sdcard.h"

//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------

SDCard sd;

SdFat SDCard::sd;
File SDCard::root;

char SDCard::sd_inserted;
char SDCard::sd_printing_now;
char SDCard::sd_printing_paused;

File SDCard::sd_print_file;
float SDCard::sd_percent_complete;
long SDCard::sd_file_size;
long SDCard::sd_bytes_read;

char SDCard::buffer[PARSER_BUFFER_LENGTH];
uint8_t SDCard::bufferPos;

//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------

// initialize the SD card and print some info.
void SDCard::setup() {
  pinMode(SDSS, OUTPUT);
  pinMode(SDCARDDETECT, INPUT);
  digitalWrite(SDCARDDETECT, HIGH);

  sd_inserted         = false;
  sd_printing_now     = false;
  sd_percent_complete = 0;
  bufferPos=0;
  check();
}

// Load the SD card and read some info about it
void SDCard::load_card() {
  sd.begin(SDSS);
  root.open("/");
}

// Check if the SD card has been added or removed
void SDCard::check() {
  int state = (digitalRead(SDCARDDETECT) == LOW);
  if (sd_inserted != state) {
    SERIAL_ECHOPGM("SD is ");
    if (!state) {
      SERIAL_ECHOLNPGM("removed");
      if (sd_printing_now) {
        // TODO: uh oh!  Please reinsert SD card!
        // TODO: lift pen if needed
        // TODO: flag waiting for SD return
      }
      sd_printing_now = false;
    } else {
      SERIAL_ECHOLNPGM("added");
      load_card();
      // TODO: is flag waiting for SD return active?
      // TODO: lower pen if needed
      // TODO: resume printing
    }
    sd_inserted = state;
  }

  // read one line from the file.  don't read too fast or the LCD will appear to hang.
  if (sd_printing_now && !sd_printing_paused && planner.movesFree()) {
    int c;
    while (sd_print_file.peek() != -1) {
      c = sd_print_file.read();
      // sd_bytes_read++;
      if (c == '\r') continue;
      if (parser.sofar < PARSER_BUFFER_LENGTH) {
        buffer[bufferPos++]=c;
        if(c=='\0'||c=='\n') {
          buffer[bufferPos]=0;
          parser.ringBuffer.waitToAdd(buffer);

          // update the % visible on the LCD.
          sd_percent_complete = 100.0 * (float)sd_bytes_read / (float)sd_file_size;
          // echo command?
          // SERIAL_ECHOLN(buffer);

          bufferPos=0;
          // quit this loop so we can update the LCD and listen for commands from the laptop (if any)
          break;
        }
      }
    }

    if (sd_print_file.peek() == -1) {
      SERIAL_ECHOLNPGM("EOF Drawing done.");
      sd_print_file.close();
      sd_printing_now = false;
      if (sd_inserted) {
        // TODO: probably file ended OK.
      } else {
        // TODO: SD card remove unexpected?
      }
    }
  }
}


void SDCard::StartPrintingFile(File toPrint) {
  sd_print_file = toPrint;

  // count the number of lines (\n characters) for displaying % complete.
  sd_file_size        = sd_print_file.fileSize();
  sd_bytes_read       = 0;
  sd_percent_complete = 0;

  // return to start
  sd_print_file.rewind();

  sd_printing_now    = true;
  sd_printing_paused = false;
}

void SDCard::listFiles() {
  if (!sd_inserted) return;

  root.rewindDirectory();
  SdFile entry;
  char filename[32];
  while (entry.openNext(&root)) {
    if (!entry.isSubDir() && !entry.isHidden()) {
      entry.getName(filename, 32);
      SERIAL_ECHOLN(filename);
    }
    entry.close();
  }
}

#endif  // HAS_SD

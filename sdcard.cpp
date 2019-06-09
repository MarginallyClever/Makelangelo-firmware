//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#include "configure.h"

#include "sdcard.h"

//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
#ifdef HAS_SD

File root;
char sd_inserted;
char sd_printing_now;
char sd_printing_paused;
File sd_print_file;
float sd_percent_complete;
long sd_file_size;
long sd_bytes_read;

#endif



//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------

// initialize the SD card and print some info.
void SD_setup() {
#ifdef HAS_SD
  pinMode(SDSS, OUTPUT);
  pinMode(SDCARDDETECT,INPUT);
  digitalWrite(SDCARDDETECT,HIGH);

  sd_inserted = false;
  sd_printing_now=false;
  sd_percent_complete=0;
  SD_check();
#endif  // HAS_SD
}


// Load the SD card and read some info about it
void SD_load_card() {
#ifdef HAS_SD
  SD.begin(SDSS);
  root = SD.open("/");
#endif
}


// Check if the SD card has been added or removed
void SD_check() {
#ifdef HAS_SD
  int state = (digitalRead(SDCARDDETECT) == LOW);
  if(sd_inserted != state) {
    Serial.print("SD is ");
    if(!state) {
      Serial.println(F("removed"));
      if(sd_printing_now) {
        // TODO: uh oh!  Please reinsert SD card!
        // TODO: lift pen if needed
        // TODO: flag waiting for SD return
      }
      sd_printing_now=false;
    } else {
      Serial.println(F("added"));
      SD_load_card();
      // TODO: is flag waiting for SD return active?
        // TODO: lower pen if needed
        // TODO: resume printing
    }
    sd_inserted = state;
  }

  // read one line from the file.  don't read too fast or the LCD will appear to hang.
  if(sd_printing_now==true && sd_printing_paused==false && segment_buffer_full()==false ) {
    int c;
    while(sd_print_file.peek() != -1) {
      c=sd_print_file.read();
      sd_bytes_read++;
      if(c=='\r') continue;
      if(sofar<MAX_BUF) {
        serialBuffer[sofar++]=c;
      }/*
      if(c==';') {
        // eat to the end of the line
        while(sd_print_file.peek() != -1) {
          c=sd_print_file.read();
          sd_bytes_read++;
          if(c=='\n' || c=='\r') break;
        }
      }*/
      if(c=='\n') {
        // update the % visible on the LCD.
        sd_percent_complete = (float)sd_bytes_read * 100.0 / (float)sd_file_size;

        // end string
        serialBuffer[sofar-1]=0;
        // print for our benefit
        //Serial.println(serialBuffer);
        // process command
        processCommand();
        // reset buffer for next line
        parser_ready();
        // quit this loop so we can update the LCD and listen for commands from the laptop (if any)
        break;
      }
    }

    if(sd_print_file.peek() == -1) {
      Serial.println("EOF Drawing done.");
      sd_print_file.close();
      sd_printing_now=false;
      if(sd_inserted) {
        // TODO: probably file ended OK.
      } else {
        // TODO: SD card remove unexpected?
      }
    }
  }
#endif // HAS_SD
}


void SD_StartPrintingFile(const char *filename) {
#ifdef HAS_SD
  sd_print_file=SD.open(filename);
  if(!sd_print_file) {
    Serial.print(F("File "));
    Serial.print(filename);
    Serial.println(F(" not found."));
    return;
  }

  // count the number of lines (\n characters) for displaying % complete.
  sd_file_size=sd_print_file.size();
  sd_bytes_read=0;
  sd_percent_complete=0;

  // return to start
  sd_print_file.seek(0);

  sd_printing_now=true;
  sd_printing_paused=false;
#endif
}


void SD_listFiles() {
#ifdef HAS_SD
  if (!sd_inserted) return;

  root.rewindDirectory();
  while (1) {
    File entry =  root.openNextFile();
    if (!entry) {
      // no more files, return to the first file in the directory
      break;
    }
    if (!entry.isDirectory() && entry.name()[0] != '_') {
      Serial.println(entry.name());
    }
    entry.close();
  }
#endif
}

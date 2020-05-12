
#include "configure.h"
#include "eeprom.h"
#include "lcd.h"
#include "sdcard.h"


// GLOBALS

Parser parser;


uint8_t absolute_mode = 1; // absolute or incremental programming mode?
uint8_t current_tool = 0;


/**
   Compare two floats to the first decimal place.
   return true when abs(a-b)<0.1
*/
boolean equalEpsilon(float a, float b) {
  int aa = floor(a * 10);
  int bb = floor(b * 10);
  //Serial.print("aa=");        Serial.print(aa);
  //Serial.print("\tbb=");      Serial.print(bb);
  //Serial.print("\taa==bb ");  Serial.println(aa==bb?"yes":"no");

  return aa == bb;
}


void Parser::start() {
  // start communications
  Serial.begin(BAUD);
  // clear input buffer
  sofar = 0;
}


/**
   Look for character /code/ in the buffer and read the float that immediately follows it.
   @return the value found.  If nothing is found, /val/ is returned.
   @input code the character to look for.
   @input val the return value if /code/ is not found.
*/
float Parser::parseNumber(char code, float val) {
  char *ptr = serialBuffer; // start at the beginning of buffer
  char *finale = serialBuffer + sofar;
  for (ptr = serialBuffer; ptr < finale; ++ptr) { // walk to the end
    if (*ptr == ';') break;
    if (toupper(*ptr) == code) { // if you find code on your walk,
      return atof(ptr + 1); // convert the digits that follow into a float and return it
    }
  }
  return val;  // end reached, nothing found, return default val.
}


// @return 1 if the character is found in the serial buffer, 0 if it is not found.
uint8_t Parser::hasGCode(char code) {
  char *ptr = serialBuffer; // start at the beginning of buffer
  char *finale = serialBuffer + sofar;
  for (ptr = serialBuffer; ptr < finale; ++ptr) { // walk to the end
    if (*ptr == ';') break;
    if (toupper(*ptr) == code) { // if you find code on your walk,
      return 1;  // found
    }
  }
  return 0;  // not found
}


// @return 1 if CRC ok or not present, 0 if CRC check fails.
char Parser::checkLineNumberAndCRCisOK() {
  // is there a line number?
  if (serialBuffer[0] == 'N') { // line number must appear first on the line
    int32_t cmd = parseNumber('N', -1);
    if( cmd != lineNumber ) {
      // wrong line number error
      Serial.print(F("BADLINENUM "));
      Serial.println(lineNumber);
      return 0;
    }

    // next time around, wait for the next line number.
    lineNumber++;
  }

  // is there a checksum?
  int found=-1;
  int i;
  for (i = strlen(serialBuffer) - 1; i >= 0; --i) {
    if (serialBuffer[i] == '*') {
      found=i;
      break;
    }
  }

  if(found==-1) {
    Serial.println("NOCHECKSUM");
    return 0;
  }
  
  // yes.  is it valid?
  int checksum = 0;
  int c;
  for (c = 0; c < i; ++c) {
    checksum = ( checksum ^ serialBuffer[c] ) & 0xFF;
  }
  c++; // skip *
  int against = strtod(serialBuffer + c, NULL);
  if ( checksum != against ) {
    Serial.print("BADCHECKSUM calc=");
    Serial.print(checksum);
    Serial.print(" sent=");
    Serial.println(against);
    return 0;
  }

  // remove checksum
  serialBuffer[i] = 0;
 
  return 1;  // ok!
}


/**
   prepares the input buffer to receive a new message and tells the serial connected device it is ready for more.
*/
void Parser::ready() {
  Serial.print(F("\n> "));  // signal ready to receive input
  lastCmdTimeMs = millis();
}


// See also http://www.marginallyclever.com/2011/10/controlling-your-arduino-through-the-serial-monitor/
void Parser::update() {
  // listen for serial commands
  if(Serial.available() > 0) {
    char c = Serial.read();
    //Serial.print(c);
    if (sofar < MAX_BUF) serialBuffer[sofar++] = c;
    if (c == '\r' || c == '\n') {
      serialBuffer[sofar - 1] = 0;

      // echo confirmation
      //if(MUST_ECHO) 
      Serial.println(serialBuffer);

      // do something with the command
      processCommand();
      // clear input buffer
      sofar = 0;
      // go again
      ready();
    }
  }

#ifdef HAS_WIFI
  int packetSize = port.parsePacket();
  if (packetSize) {
    int len = port.read(serialBuffer, MAX_BUF);
    sofar = len;
    if (len > 0) serialBuffer[len - 1] = 0;
    Serial.println(serialBuffer);
    processCommand();
    port.beginPacket(port.remoteIP(), port.remotePort());
    port.write("ok\r\n");
    port.endPacket();
  }
#endif  // HAS_WIFI
}



/**
   process commands in the serial receive buffer
*/
void Parser::processCommand() {
  if( serialBuffer[0] == '\0' || serialBuffer[0] == ';' ) return; // blank lines

  if(IS_STRICT && !checkLineNumberAndCRCisOK()) return; // message garbled

  // remove any trailing semicolon.
  int last = strlen(serialBuffer) - 1;
  if( serialBuffer[last] == ';') serialBuffer[last] = 0;

  if( !strncmp(serialBuffer, "UID", 3) ) {
    robot_uid = atoi(strchr(serialBuffer, ' ') + 1);
    saveUID();
  }

  int16_t cmd;

  // M codes
  if(hasGCode('M')) {
    cmd = parseNumber('M', -1);
    switch (cmd) {
      case   6:  M6();  break;
      case  17:  motor_engage();  break;
      case  18:  motor_disengage();  break;
#ifdef HAS_SD
      case  20:  SD_listFiles();  break;
#endif
      case  42:  M42();  break;
      case 100:  M100();  break;
      case 101:  M101();  break;
      case 110:  lineNumber = parseNumber('N', lineNumber);  break;
      case 114:  M114();  break;
      case 117:  M117();  break;
      case 203:  M203();  break;
      case 205:  M205();  break;
      case 226:  M226();  break;
      case 300:  M300();  break;
      default:   break;
    }
    return;
  }

  // machine style-specific codes
  if(hasGCode('D')) {
    cmd = parseNumber('D', -1);
    switch (cmd) {
      case  0:  D0();  break;
  #ifdef HAS_SD
  //    case  4:  SD_StartPrintingFile(strchr(serialBuffer, ' ') + 1);  break;
  #endif
      case  5:  D5();  break;
      case  6:  D6();  break;
      case  7:  D7();  break;
      case  8:  D8();  break;
      case  9:  saveCalibration();  break;
      case 10:  // get hardware version
        Serial.print(F("D10 V"));
        Serial.println(MACHINE_HARDWARE_VERSION);
        break;
#if MACHINE_STYLE == POLARGRAPH
#if MACHINE_HARDWARE_VERSION == MAKELANGELO_6
      case 11:  makelangelo6Setup();  break;
#endif
#if MACHINE_HARDWARE_VERSION == MAKELANGELO_5
      case 11:  makelangelo5Setup();  break;
#endif
#if MACHINE_HARDWARE_VERSION == MAKELANGELO_3_3
      case 11:  makelangelo33Setup();  break;
#endif
      //case 12:  recordHome();  break;
#endif
#ifdef MACHINE_HAS_LIFTABLE_PEN
      case 13:  setPenAngle(parseNumber('Z', axies[2].pos));  break;
#endif
      case 14:  // get machine style
        Serial.print(F("D14 "));
        Serial.println(MACHINE_STYLE_NAME);
        break;
#if MACHINE_STYLE == STEWART
      case 15:  stewartDemo();  break;
#endif
#if MACHINE_STYLE == SIXI
      case 15:  sixiDemo();  break;
#endif
      case 16:  D16();  break;
#if MACHINE_STYLE == SIXI
      case 17:  D17();  break;
      case 18:  D18();  break;
      case 19:  positionErrorFlags ^= POSITION_ERROR_FLAG_CONTINUOUS;  break; // toggle
      case 20:  positionErrorFlags &= 0xffff ^ (POSITION_ERROR_FLAG_ERROR | POSITION_ERROR_FLAG_FIRSTERROR);  break; // off
      case 21:  positionErrorFlags ^= POSITION_ERROR_FLAG_ESTOP;  break; // toggle ESTOP
      case 22:  sixiResetSensorOffsets();  break;
#endif
      case 23:  D23();  break;
      default:  break;
    }
    return;
  }

  // no M or D commands were found.  This is probably a G-command.
  // G codes
  cmd = parseNumber('G', lastGcommand);
  lastGcommand = -1;
  switch (cmd) {
    case  0:
    case  1:  G01();  lastGcommand = cmd;  break;
    case  2:  G02(1);  lastGcommand = cmd;  break; // clockwise
    case  3:  G02(0);  lastGcommand = cmd;  break; // counter-clockwise
    case  4:  G04();  break;
    case 28:  robot_findHome();  break;
#if MACHINE_STYLE == POLARGRAPH
    //case 29:  calibrateBelts();  break;
#endif
    case 90:  absolute_mode = 1;  break; // absolute mode
    case 91:  absolute_mode = 0;  break; // relative mode
    case 92:  G92();  break;
    default:  break;
  }
}


// D commands


/**
   D0 [Lnn] [Rnn] [Unn] [Vnn] [Wnn] [Tnn]
   Jog each motor nn steps.
   I don't know why the latter motor names are UVWT.
*/
void Parser::D0() {
  int i, j, amount;

  motor_engage();
  findStepDelay();

  for (i = 0; i < NUM_MOTORS; ++i) {
    if (MotorNames[i] == 0) continue;
    amount = parseNumber(MotorNames[i], 0);
    if (amount != 0) {
      Serial.print(F("Moving "));
      Serial.print(MotorNames[i]);
      Serial.print(F(" ("));
      Serial.print(i);
      Serial.print(F(") "));
      Serial.print(amount);
      Serial.print(F(" steps. Dir="));
      Serial.print(motors[i].dir_pin);
      Serial.print(F(" Step="));
      Serial.print(motors[i].step_pin);
      Serial.print('\n');

      int x = amount < 0 ? STEPPER_DIR_HIGH  : STEPPER_DIR_LOW;
      digitalWrite(motors[i].dir_pin, x);

      amount = abs(amount);
      for (j = 0; j < amount; ++j) {
        digitalWrite(motors[i].step_pin, HIGH);
        digitalWrite(motors[i].step_pin, LOW);
        pause(step_delay);
      }
    }
  }
}


/**
   D5
   report current firmware version
*/
void Parser::D5() {
  char versionNumber = loadVersion();

  Serial.print(F("Firmware v"));
  Serial.println(versionNumber, DEC);
}


/**
   D6 [Xnnn] [Ynnn] [Znnn] [Unnn] [Vnnn] [Wnnn]
   Set home position for each axis.
*/
void Parser::D6() {
  int i;
  float homePos[NUM_AXIES];
  for (i = 0; i < NUM_AXIES; ++i) {
    homePos[i] = parseNumber(AxisNames[i], axies[i].homePos);
  }
  setHome(homePos);
}


/**
   D7 [Lnnn] [Rnnn]
   Set calibration length of each belt
*/
void Parser::D7() {
  calibrateLeft = parseNumber('L', calibrateLeft);
  calibrateRight = parseNumber('R', calibrateRight);
  D8();
}


/**
   D8
   Report calibration values for left and right belts
*/
void Parser::D8() {
  Serial.print(F("D8 L"));
  Serial.print(calibrateLeft);
  Serial.print(F(" R"));
  Serial.println(calibrateRight);
}



/**
   D16 X<jerk> Y<jerk> Z<jerk> U<jerk> V<jerk> W<jerk>
   set axis n to feedrate m and jerk o.
*/
void Parser::D16() {
  float f;

  Serial.print("D16 X");  Serial.print(max_feedrate_mm_s[0]);
  f = parseNumber('X', max_feedrate_mm_s[0]);  max_feedrate_mm_s[0] = max(f, (float)MIN_FEEDRATE);
#if NUM_AXIES > 1
  f = parseNumber('Y', max_feedrate_mm_s[1]);  max_feedrate_mm_s[1] = max(f, (float)MIN_FEEDRATE);
  Serial.print(" Y");  Serial.print(max_feedrate_mm_s[1]);
#endif
#if NUM_AXIES > 2
  f = parseNumber('Z', max_feedrate_mm_s[2]);  max_feedrate_mm_s[2] = max(f, (float)MIN_FEEDRATE);
  Serial.print(" Z");  Serial.print(max_feedrate_mm_s[2]);
#endif
#if NUM_AXIES > 3
  f = parseNumber('U', max_feedrate_mm_s[3]);  max_feedrate_mm_s[3] = max(f, (float)MIN_FEEDRATE);
  Serial.print(" U");  Serial.print(max_feedrate_mm_s[3]);
#endif
#if NUM_AXIES > 4
  f = parseNumber('V', max_feedrate_mm_s[4]);  max_feedrate_mm_s[4] = max(f, (float)MIN_FEEDRATE);
  Serial.print(" V");  Serial.print(max_feedrate_mm_s[4]);
#endif
#if NUM_AXIES > 5
  f = parseNumber('W', max_feedrate_mm_s[5]);  max_feedrate_mm_s[5] = max(f, (float)MIN_FEEDRATE);
  Serial.print(" W");  Serial.println(max_feedrate_mm_s[5]);
#endif
}


#if MACHINE_STYLE == SIXI
/**
   D17 report the 6 axis sensor values from the Sixi robot arm.
*/
void Parser::D17() {
  Serial.print(F("D17"));
  for (int i = 0; i < 6; ++i) {
    Serial.print('\t');
    Serial.print(sensorAngles[i], 2);
  }
  /*
    if(current_segment==last_segment) {
    // report estimated position
    Serial.print(F("\t-\t"));

    working_seg = get_current_segment();
    for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
      //float diff = working_seg->a[i].expectedPosition - sensorAngles[i];
      //Serial.print('\t');
      //Serial.print(abs(diff),3);
      Serial.print('\t');
      Serial.print(working_seg->a[i].expectedPosition,2);
    }
    }*/

  Serial.print('\t');
  //Serial.print(((positionErrorFlags&POSITION_ERROR_FLAG_CONTINUOUS)!=0)?'+':'-');
  Serial.print(((positionErrorFlags & POSITION_ERROR_FLAG_ERROR) != 0) ? '+' : '-');
  //Serial.print(((positionErrorFlags&POSITION_ERROR_FLAG_FIRSTERROR)!=0)?'+':'-');
  //Serial.print(((positionErrorFlags&POSITION_ERROR_FLAG_ESTOP)!=0)?'+':'-');
  Serial.println();
}
#endif


#if MACHINE_STYLE == SIXI
/**
   D18 copy sensor values to motor step positions.
*/
void Parser::D18() {
  wait_for_empty_segment_buffer();
  float a[NUM_AXIES];
  int i, j;
  int numSamples = 10;

  for (j = 0; j < NUM_AXIES; ++j) a[j] = 0;

  // assert(NUM_SENSORS <= NUM_AXIES);

  for (i = 0; i < numSamples; ++i) {
    sensorUpdate();
    for (j = 0; j < NUM_SENSORS; ++j) {
      a[j] += sensorAngles[j];
    }
  }
  for (j = 0; j < NUM_SENSORS; ++j) {
    a[j] /= (float)numSamples;
  }

  teleport(a);
}
#endif



/**
   D23 report home values
*/
void Parser::D23() {
  Serial.print(F("D23 "));
  for (int i = 0; i < NUM_AXIES; ++i) {
    Serial.print(AxisNames[i]);
    Serial.print(axies[i].homePos);
    Serial.print(' ');
  }
  Serial.println();
}


// G commands


/**
   G0-G1 [Xnnn] [Ynnn] [Znnn] [Unnn] [Vnnn] [Wnnn] [Ann] [Fnn]
   straight lines.  distance in mm.
*/
void Parser::G01() {
  acceleration = parseNumber('A', acceleration);
  acceleration = min(max(acceleration, (float)MIN_ACCELERATION), (float)MAX_ACCELERATION);
  float f = parseNumber('F', feed_rate);
  f = min(max(f, (float)MIN_FEEDRATE), (float)MAX_FEEDRATE);

  int i;
  float pos[NUM_AXIES];
  for (i = 0; i < NUM_AXIES; ++i) {
    float p = axies[i].pos;
    pos[i] = parseNumber(AxisNames[i], (absolute_mode ? p : 0)) + (absolute_mode ? 0 : p);
  }

  lineSafe( pos, f );
}


/**
   G2-G3 [Xnnn] [Ynnn] [Ann] [Fnn] [Inn] [Jnn]
   arcs in the XY plane
   @param clockwise (G2) 1 for cw, (G3) 0 for ccw
*/
void Parser::G02(int clockwise) {
  acceleration = parseNumber('A', acceleration);
  acceleration = min(max(acceleration, (float)MIN_ACCELERATION), (float)MAX_ACCELERATION);
  float f = parseNumber('F', feed_rate);
  f = min(max(f, (float)MIN_FEEDRATE), (float)MAX_FEEDRATE);

  int i;
  float pos[NUM_AXIES];
  for (i = 0; i < NUM_AXIES; ++i) {
    float p = axies[i].pos;
    pos[i] = parseNumber(AxisNames[i], (absolute_mode ? p : 0)) + (absolute_mode ? 0 : p);
  }

  float p0 = axies[0].pos;
  float p1 = axies[1].pos;
  arc(parseNumber('I', (absolute_mode ? p0 : 0)) + (absolute_mode ? 0 : p0),
      parseNumber('J', (absolute_mode ? p1 : 0)) + (absolute_mode ? 0 : p1),
      pos,
      clockwise,
      f );
}


/**
   G4 [Snn] [Pnn]
   Wait S milliseconds and P seconds.
*/
void Parser::G04() {
  wait_for_empty_segment_buffer();
  float delayTime = parseNumber('S', 0) + parseNumber('P', 0) * 1000.0f;
  pause(delayTime);
}


/**
   G92 [Xnnn] [Ynnn] [Znnn] [Unnn] [Vnnn] [Wnnn]
   Teleport mental position
*/
void Parser::G92() {
  int i;
  float pos[NUM_AXIES];
  for (i = 0; i < NUM_AXIES; ++i) {
    float p = axies[i].pos;
    pos[i] = parseNumber(AxisNames[i], (absolute_mode ? p : 0)) + (absolute_mode ? 0 : p);
  }
  teleport(pos);
}


// M commands



/**
   M6 [Tnnn]
   Change the currently active tool
*/
void Parser::M6() {
  int tool_id = parseNumber('T', current_tool);
  if (tool_id < 0) tool_id = 0;
  if (tool_id >= NUM_TOOLS) tool_id = NUM_TOOLS - 1;
  current_tool = tool_id;
}


/**
   M42 P[a] S[b]
   Set digital pin a to state b (1 or 0).
   default pin is LED_BUILTIN.  default state is LOW
*/
void Parser::M42() {
  int pin = parseNumber('P', LED_BUILTIN);
  int newState = parseNumber('S', 0);
  digitalWrite(pin, newState ? HIGH : LOW);
}


void Parser::sayBuildDateAndTime() {
  Serial.print(F("Built "));
  Serial.println(__DATE__ " " __TIME__);
}


void Parser::sayModelAndUID() {
  Serial.print(F("I AM "));
  Serial.print(MACHINE_STYLE_NAME);
  Serial.print(F(" #"));
  Serial.println(robot_uid);
}


/**
   M100
   Print a M100ful message to serial.  The first line must never be changed to play nice with the JAVA software.
*/
void Parser::M100() {
  Serial.print(F("\n\nHELLO WORLD! "));
  sayModelAndUID();
  D5();
  sayBuildDateAndTime();
  Serial.println(F("Please see http://makelangelo.com/ for more information."));
  Serial.println(F("Try these (with a newline): G00,G01,G02,G03,G04,G28,G90,G91,G92,M18,M101,M100,M114"));
#ifdef HAS_WIFI
  // Print the IP address
  Serial.print("Use this URL to connect: http://");
  Serial.print(WiFi.softAPIP());
  Serial.print(':');
  Serial.print(localPort);
  Serial.println('/');
#endif  // HAS_WIFI
#ifdef HAS_LCD
  Serial.println(F("Has LCD"));
#endif
}


/**
   M101 Annn Tnnn Bnnn
   Change axis A limits to max T and min B.
   look for change to dimensions in command, apply and save changes.
*/
void Parser::M101() {
  int axisNumber = parseNumber('A', -1);
  if (axisNumber >= 0 && axisNumber < NUM_AXIES) {
    float newT = parseNumber('T', axies[axisNumber].limitMax);
    float newB = parseNumber('B', axies[axisNumber].limitMin);
    boolean changed = false;

    if (!equalEpsilon(axies[axisNumber].limitMax, newT)) {
      axies[axisNumber].limitMax = newT;
      changed = true;
    }
    if (!equalEpsilon(axies[axisNumber].limitMin, newB)) {
      axies[axisNumber].limitMin = newB;
      changed = true;
    }
    if (changed == true) {
      saveLimits();
    }
  }


  Serial.print(F("("));

  int i;
  for (i = 0; i < NUM_AXIES; ++i) {
    Serial.print(axies[i].limitMin);
    if (i < NUM_AXIES - 1)  Serial.print(',');
  }

  Serial.print(F(") - ("));

  for (i = 0; i < NUM_AXIES; ++i) {
    Serial.print(axies[i].limitMax);
    if (i < NUM_AXIES - 1)  Serial.print(',');
  }

  Serial.print(F(")\n"));
}


/**
   M114
   Print the X,Y,Z, feedrate, acceleration, and home position
*/
void Parser::M114() {
  wait_for_empty_segment_buffer();

  int i;
  for (i = 0; i < NUM_AXIES; ++i) {
    Serial.print(AxisNames[i]);
    Serial.print(axies[i].pos);
    Serial.print(' ');
  }

  Serial.print(F("F"));
  Serial.print(feed_rate);
  Serial.print(F("mm/s"));

  Serial.print(F(" A"));
  Serial.println(acceleration);
}


/**
   M117 [string]
   Display string on the LCD panel.  Command is ignored if there is no LCD panel.
*/
void Parser::M117() {
#ifdef HAS_LCD
  // find M
  uint16_t i = 0;
  // skip "N*** M117 "
  while(serialBuffer[i]!='M') ++i;
  i+=5;
  // anything left?
  if (i >= strlen(serialBuffer)) {
    // no message
    LCD_setStatusMessage(0);
    return;
  }
  
  // read in any remaining message
  char message[M117_MAX_LEN];
  char *buf = serialBuffer + i;
  i = 0;
  while (isPrintable(*buf) && (*buf) != '\r' && (*buf) != '\n' && i < M117_MAX_LEN) {
    message[i++] = *buf;
    buf++;
  }
  
  // make sure the message is properly terminated
  message[i]=0;
  // update the LCD.
  LCD_setStatusMessage(message);
#endif  // HAS_LCD
}


/**
   M203 X2000 Y5000 Z200 etc...
   adjust the max feedrate of each axis
*/
void Parser::M203() {
  int i;
  Serial.print(F("M203 "));
  for (i = 0; i < NUM_MOTORS + NUM_SERVOS; ++i) {
    max_feedrate_mm_s[i] = parseNumber(MotorNames[i], max_feedrate_mm_s[i]);
    Serial.print(MotorNames[i]);
    Serial.print(max_feedrate_mm_s[i]);
    Serial.print(' ');
  }
  Serial.println();
}


/**
   M205 X<jerk> Y<jerk> Z<jerk> U<jerk> V<jerk> W<jerk> B<us>
   adjust max jerk
*/
void Parser::M205() {
  float f;
  f = parseNumber('X', max_jerk[0]);  max_jerk[0] = max(min(f, (float)MAX_JERK), (float)0);
#if NUM_AXIES>1
  f = parseNumber('Y', max_jerk[1]);  max_jerk[1] = max(min(f, (float)MAX_JERK), (float)0);
#endif
#if NUM_AXIES>2
  f = parseNumber('Z', max_jerk[2]);  max_jerk[2] = max(min(f, (float)MAX_JERK), (float)0);
#endif
#if NUM_AXIES>3
  f = parseNumber('U', max_jerk[3]);  max_jerk[3] = max(min(f, (float)MAX_JERK), (float)0);
#endif
#if NUM_AXIES>4
  f = parseNumber('V', max_jerk[4]);  max_jerk[4] = max(min(f, (float)MAX_JERK), (float)0);
#endif
#if NUM_AXIES>5
  f = parseNumber('W', max_jerk[5]);  max_jerk[5] = max(min(f, (float)MAX_JERK), (float)0);
#endif
  f = parseNumber('B', min_segment_time_us);  min_segment_time_us = max(min(f, 1000000), (float)0);
  Serial.print("M205 X");  Serial.print(max_jerk[0]);
  Serial.print(" Y");  Serial.print(max_jerk[1]);
  Serial.print(" Z");  Serial.print(max_jerk[2]);
  Serial.print(" U");  Serial.print(max_jerk[3]);
  Serial.print(" V");  Serial.print(max_jerk[4]);
  Serial.print(" W");  Serial.print(max_jerk[5]);
  Serial.print(" B");  Serial.println(min_segment_time_us);
}


/**
   M226 P[a] S[b]
   Wait for pin a to be in state b (1 or 0).
   If there is an LCD and P or S are missing, wait for user to press click wheel on LCD.
   If there is no LCD, P must be specified.
*/
void Parser::M226() {
#ifdef HAS_LCD
  int pin = parseNumber('P', BTN_ENC);
#else
  int pin = parseNumber('P', -1);
#endif
  if (pin == -1) return; // no pin specified.

  int oldState = parseNumber('S', -1);
  if (oldState == -1) {
    // default: assume the pin is not in the requested state
    oldState = digitalRead(pin);
  } else {
    // 0 for HIGH, anything else for LOW
    oldState = (oldState == 0) ? HIGH : LOW;
  }
  Serial.print("pausing");
#ifdef HAS_SD
  sd_printing_paused = true;
#endif

  // while pin is in oldState (opposite of state for which we are waiting)
  while (digitalRead(pin) == oldState) {
    meanwhile();
#ifdef HAS_SD
    //SD_check();
#endif
#ifdef HAS_LCD
    //LCD_update();  // causes menu to change when we don't want it to change.
#endif
    //Serial.print(".");
  }

#ifdef HAS_SD
  sd_printing_paused = false;
#endif

  Serial.println(" ended.");
}


/**
   M300 S[a] P[b]
   play frequency a for b milliseconds
*/
void Parser::M300() {
#ifdef HAS_LCD
  int ms = parseNumber('P', 250);
  //int freq = parseNumber('S', 60);

  digitalWrite(BEEPER, HIGH);
  delay(ms);
  digitalWrite(BEEPER, LOW);
#endif
}

#include "configure.h"
#include "lcd.h"
#include "sdcard.h"

// GLOBALS

Parser parser;

uint8_t absolute_mode = 1;  // absolute or incremental programming mode?
uint8_t current_tool  = 0;

/**
   Compare two floats to the first decimal place.
   return 1 when abs(a-b)<0.1
*/
uint8_t equalEpsilon(float a, float b) {
  int aa = floorf(a * 10);
  int bb = floorf(b * 10);
  // MYSERIAL1.print("aa=");        MYSERIAL1.print(aa);
  // MYSERIAL1.print("\tbb=");      MYSERIAL1.print(bb);
  // MYSERIAL1.print("\taa==bb ");  MYSERIAL1.println(aa==bb?"yes":"no");

  return aa == bb;
}

void Parser::start() {
  // start communications
  MYSERIAL1.begin(BAUD);
  // clear input buffer
  sofar = 0;

  ringBuffer.clear();

#ifdef HAS_WIFI
  // Start WIFI
  WiFi.mode(WIFI_AP);
  MYSERIAL1.println(WiFi.softAP(SSID_NAME, SSID_PASS) ? "WIFI OK" : "WIFI FAILED");
  MYSERIAL1.println(port.begin(localPort) ? "UDP OK" : "UDP FAILED");
  // Print the IP address
  MYSERIAL1.print("Use this URL to connect: http://");
  MYSERIAL1.print(WiFi.softAPIP());
  MYSERIAL1.print(':');
  MYSERIAL1.print(localPort);
  MYSERIAL1.println('/');
#endif  // HAS_WIFI
}

float Parser::parseFloat(char *p) {
  char *ptr = p;  // start at the beginning of buffer
  for(;;) {  // walk to the end
    char c = *ptr;
    if(c==';'||c=='\0'||c==' ') break;
    if(c=='E'||c=='c') {
      *ptr = '\0';
      const float ret = atof(p);
      *ptr = c;
      return ret;
    }
    ++ptr;
  }
  float g = atof(p);  // convert the digits that follow into a float and return it
  return g;
}

float Parser::parseFloat(const char code,float valueIfNotFound) {
  int8_t n = hasGCode(currentCommand,code);
  if(n>=0) return parseFloat(currentCommand+n+1);
  return valueIfNotFound;
}

int32_t Parser::parseInt(const char code,int32_t valueIfNotFound) {
  int8_t n = hasGCode(currentCommand,code);
  if(n>=0) return parseInt(currentCommand+n+1);
  return valueIfNotFound;
}

int8_t Parser::hasGCode(char *p,const char code) {
  char *ptr = p;  // start at the beginning of buffer
  for(;;) {  // walk to the end
    char c = *ptr;
    if(c==';' || c=='\0') break;
    if(toupper(c) == code) {
      return (uint8_t)(ptr-p);
    }
    ptr++;
  }
  return -1;  // not found
}

int8_t Parser::hasGCode(const char code) {
  return hasGCode(currentCommand,code);
}

// @return 1 if CRC ok or not present, 0 if CRC check fails.
char Parser::checkLineNumberAndCRCisOK() {
  // is there a line number?
  if(*serialBuffer == 'N') {  // line number must appear first on the line
    int32_t cmd = parseInt(serialBuffer+1);
    if(cmd != lineNumber) {
      // wrong line number error
      SERIAL_ECHOLNPAIR("BADLINENUM ",lineNumber);
      return 0;
    }
  } else if(IS_STRICT) {
    SERIAL_ECHOLNPAIR("NOLINENUM ",lineNumber);
    return 0;
  }

  // is there a checksum?
  int found = -1;
  int i;
  for (i = strlen(serialBuffer) - 1; i >= 0; --i) {
    if(serialBuffer[i] == '*') {
      found = i;
      break;
    }
  }

  if(IS_STRICT && found == -1) {
    SERIAL_ECHOLNPAIR("NOCHECKSUM ",lineNumber);
    return 0;
  }

  // yes.  is it valid?
  int checksum = 0;
  int c;
  for (c = 0; c < i; ++c) {
    checksum = (checksum ^ serialBuffer[c]) & 0xFF;
  }
  c++;  // skip *
  int against = strtod(serialBuffer + c, NULL);
  if(found != -1 && checksum != against) {
    SERIAL_ECHOLNPAIR("BADCHECKSUM",lineNumber);
    return 0;
  }

  // next time around, wait for the next line number.
  lineNumber++;
  // remove checksum
  serialBuffer[i] = 0;

  return 1;  // ok!
}

/**
 * prepares the input buffer to receive a new message and tells the MYSERIAL1 connected device it is ready for more.
 */
void Parser::ready() {
  SERIAL_ECHOPGM("\n> ");  // signal ready to receive input
  lastCmdTimeMs = millis();
}

// See also http://www.marginallyclever.com/2011/10/controlling-your-arduino-through-the-MYSERIAL1-monitor/
void Parser::update() {
  if(ringBuffer.isFull()) return;

  if(MYSERIAL1.available() > 0) {
    char c = MYSERIAL1.read();
    if(sofar < PARSER_BUFFER_LENGTH) serialBuffer[sofar++] = c;
    if(c == '\r' || c == '\n') {
      serialBuffer[sofar - 1] = 0;

      if(checkLineNumberAndCRCisOK()) {
        // echo confirmation
        if(MUST_ECHO) SERIAL_ECHO(serialBuffer);

        ringBuffer.waitToAdd(serialBuffer);
      }
      // clear input buffer
      sofar = 0;
      // go again
      ready();
    }
  }

#ifdef HAS_WIFI
  int packetSize = port.parsePacket();
  if(packetSize) {
    int len = port.read(serialBuffer, MAX_BUF);
    sofar   = len;
    if(len > 0) serialBuffer[len - 1] = 0;
    MYSERIAL1.println(serialBuffer);
    processCommand();
    port.beginPacket(port.remoteIP(), port.remotePort());
    port.write("ok\r\n");
    port.endPacket();
  }
#endif  // HAS_WIFI
}

void Parser::advance() {
  if(ringBuffer.isEmpty()) return;

  currentCommand = ringBuffer.peekNextCommand();
  if(*currentCommand == '\0' || *currentCommand == ';') {  // blank lines
    ringBuffer.advanceHead(ringBuffer.current_r,-1);
    return;
  }

  //reportQueue();
  processCommand();

  ringBuffer.advanceHead(ringBuffer.current_r,-1);
}

void Parser::reportQueue() {
  SERIAL_ECHOLNPAIR("length=",ringBuffer.length);
  SERIAL_ECHOLNPAIR("current_r=",ringBuffer.current_r);
  SERIAL_ECHOLNPAIR("current_w=",ringBuffer.current_w);
  for(uint8_t i=0;i<ringBuffer.length;++i) {
    uint8_t n = (ringBuffer.current_r+i)%RING_BUFFER_SIZE;
    SERIAL_ECHOLNPAIR("  ",ringBuffer.commands[n].buffer);
  }
}

void Parser::processCommand() {
  // eat whitespace
  while(*currentCommand==' ') currentCommand++;
  // eat line number
  if(toupper(*currentCommand)=='N' && NUMERIC(currentCommand[1])) {
    currentCommand++;
    while(NUMERIC(*currentCommand)) ++currentCommand;
    // eat whitespace
    while(*currentCommand==' ') currentCommand++;
  }

  // remove any trailing semicolon or checksum
  char *ptr = currentCommand;
  while(*ptr != '\0') {
    char c=*ptr;
    if(c==';' || c=='*') {
      *ptr=0;
      break;
    }
    ptr++;
  }

  char c = toupper(*currentCommand);

  int16_t cmd;

  if(c=='U') {
    if(!strncmp(currentCommand, "UID", 3)) {
      robot_uid = atoi(strchr(currentCommand, ' ') + 1);
      eepromManager.saveUID();
    }
    return;
  }
  
  // M codes
  if(c=='M') {
    cmd = parseInt(currentCommand+1);
    switch (cmd) {
      case   6:        M6();          break;
      case  17:        M17();         break;
      case  18:        M18();         break;
#ifdef HAS_SD
      case  20:        sd.listFiles(); break;
#endif
      case  42:        M42();         break;
      case  92:        M92();         break;
      case 100:        M100();        break;
      case 101:        M101();        break;
      case 110:        M110();        break;
      case 112:        M112();        break;
      case 114:        M114();        break;
#ifdef HAS_LCD
      case 117:        M117();        break;
#endif
      case 203:        M203();        break;
      case 205:        M205();        break;
      case 226:        M226();        break;
      case 280:        M280();        break;
      case 300:        M300();        break;
     //case 306:  M306();  break;
#if MACHINE_STYLE == SIXI
      case 428:        M428();        break;
#endif
      case 500:        M500();        break;
      case 501:        M501();        break;
      case 502:        M502();        break;
      case 503:        M503();        break;
      default:                        break;
    }
    return;
  }

  // machine style-specific codes
  if(c=='D') {
    cmd = parseInt(currentCommand+1);
    if(cmd!=-1) {
      switch (cmd) {
        case 0:        D0();        break;
  #ifdef HAS_SD
  //    case  4:  SD_StartPrintingFile(strchr(currentCommand, ' ') + 1);  break;
  #endif
        case 5:        D5();        break;
        case 6:        D6();        break;
  #if MACHINE_STYLE == POLARGRAPH
        case 7:        D7();        break;
        case 8:        D8();        break;
  #endif
        case 9:        eepromManager.saveCalibration();        break;
        case 10:       D10();        break;
  #ifdef MACHINE_HAS_LIFTABLE_PEN
        case 13:       D13();        break;
  #endif
        case 14:       D14();        break;
  #ifdef IS_STEWART_PLATFORM
        case 15:       stewartDemo();break;
  #endif
  #if MACHINE_STYLE == SIXI
        case 15:       sixiDemo();   break;
        case 17:       D17();        break;
        case 18:       D18();        break;
        case 19:       D19();        break;
        case 20:       SET_BIT_OFF(sensorManager.positionErrorFlags, POSITION_ERROR_FLAG_ERROR);        break;
        case 21:       FLIP_BIT(sensorManager.positionErrorFlags, POSITION_ERROR_FLAG_ESTOP);        break;  // toggle ESTOP
        case 23:       D23();        break;
  #endif
        case 50:       D50();        break;
        default:                     break;
      }
      return;
    }
  }

  // no M or D commands were found.  This is a G* command.
  if(c=='G') {
    cmd = parseInt(currentCommand+1);
  } else cmd = lastGcommand;

  lastGcommand = -1;
  switch (cmd) {
    case 0:
    case 1:      G01();         lastGcommand = cmd;      break;
    case 2:      G02(ARC_CW);   lastGcommand = cmd;      break;  // clockwise
    case 3:      G02(ARC_CCW);  lastGcommand = cmd;      break;  // counter-clockwise
    case 4:      G04();                                  break;
    case 28:     robot_findHome();                       break;
#if MACHINE_STYLE == POLARGRAPH
      // case 29:  calibrateBelts();  break;
#endif
    case 90:      absolute_mode = 1;      break;  // absolute mode
    case 91:      absolute_mode = 0;      break;  // relative mode
    case 92:      G92();                  break;
    default:                              break;
  }
  return;
}

// D commands

/**
   D0 [Lnn] [Rnn] [Unn] [Vnn] [Wnn] [Tnn]
   Jog each motor nn steps.
   I don't know why the latter motor names are UVWT.
*/
void Parser::D0() {
  int j, amount;

  motor.engage();

  int stepDelay = findStepDelay();
  SERIAL_ECHOLNPAIR("Step delay=",stepDelay);
  SERIAL_ECHOLNPAIR("feed rate=",desiredFeedRate);
  SERIAL_ECHOLNPAIR("UNITS_PER_STEP=",UNITS_PER_STEP);

  SERIAL_ECHOLNPAIR("STEPPER_TIMER_PRESCALE=",STEPPER_TIMER_PRESCALE);
  SERIAL_ECHOLNPAIR("HAL_TIMER_RATE=",HAL_TIMER_RATE);
  SERIAL_ECHOLNPAIR("STEPPER_TIMER_RATE=",STEPPER_TIMER_RATE);
  SERIAL_ECHOLNPAIR("STEPPER_TIMER_TICKS_PER_US=",STEPPER_TIMER_TICKS_PER_US);

  for(ALL_MOTORS(i)) {
    if(motors[i].letter == 0) continue;
    amount = parseInt(motors[i].letter, 0);
    if(amount != 0) {
      SERIAL_ECHOPAIR("Moving ",motors[i].letter);
      SERIAL_ECHOPAIR("(",i);
      SERIAL_ECHOPAIR(") ",amount);
      SERIAL_ECHOPAIR(" steps. Dir=",motors[i].dir_pin);
      SERIAL_ECHOPAIR(" Step=",motors[i].step_pin);

      int x = amount < 0 ? STEPPER_DIR_HIGH : STEPPER_DIR_LOW;
      digitalWrite(motors[i].dir_pin, x);

      amount = abs(amount);
      for (j = 0; j < amount; ++j) {
        digitalWrite(motors[i].step_pin, HIGH);
        digitalWrite(motors[i].step_pin, LOW);
        pause(stepDelay);
        if(j%100) MYSERIAL1.print('.');
        digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
      }
      SERIAL_EOL();
    }
  }
}

/**
   D5
   report current firmware version
*/
void Parser::D5() {
  char versionNumber = eepromManager.loadVersion();
  SERIAL_ECHOLNPAIR("Firmware v",versionNumber);
}

/**
   D6 [Xnnn] [Ynnn] [Znnn] [Unnn] [Vnnn] [Wnnn]
   Set home position for each axis.
*/
void Parser::D6() {
  SERIAL_ECHOPGM("D6");
  float homePos[NUM_AXIES];
  for(ALL_AXIES(i)) {
    homePos[i] = parseFloat(GET_AXIS_NAME(i), axies[i].homePos);
    SERIAL_CHAR(' ');
    SERIAL_CHAR(GET_AXIS_NAME(i));
    SERIAL_PRINT(homePos[i],2);
  }
  SERIAL_EOL();
  setHome(homePos);
}

#if MACHINE_STYLE == POLARGRAPH
/**
   D7 [Lnnn] [Rnnn]
   Set calibration length of each belt
*/
void Parser::D7() {
  calibrateLeft  = parseFloat('L', calibrateLeft);
  calibrateRight = parseFloat('R', calibrateRight);
  D8();
}
#endif

#if MACHINE_STYLE == POLARGRAPH
/**
   D8
   Report calibration values for left and right belts
*/
void Parser::D8() {
  SERIAL_ECHOPAIR("D8 L",calibrateLeft);
  SERIAL_ECHOLNPAIR(" R",calibrateRight);
}
#endif

/**
   D10
   get hardware version
*/
void Parser::D10() {
  SERIAL_ECHOLNPAIR("D10 V",MACHINE_HARDWARE_VERSION);
}

void Parser::D13() {
  motor.setPenAngle(parseFloat('Z', axies[2].pos));
}

void Parser::D14() {
  // get machine style
  SERIAL_ECHOLNPAIR("D14 ",MACHINE_STYLE_NAME);
}

#if MACHINE_STYLE == SIXI
// D17 report the 6 axis sensor values from the Sixi robot arm.
void Parser::D17() {
  SERIAL_ECHOPGM("D17"));
  for (ALL_SENSORS(i)) {
    // 360/(2^14) aka 0.02197265625deg is the minimum sensor resolution.  as such more than 3 decimal places is useless.
    SERIAL_ECHOPAIR_F('\t',WRAP_DEGREES(sensorManager.sensors[i].angle), 3);
  }

#  if NUM_SERVOS > 0
  SERIAL_ECHOPAIR_F(' ',(float)servos[0].read(), 2);
#  endif

  /*
    if(current_segment==last_segment) {
    // report estimated position
    SERIAL_ECHO("\t-\t");

    working_seg = get_current_segment();
    for (ALL_SENSORS(i)) {
      //float diff = working_seg->a[i].expectedPosition - sensorAngles[i];
      //SERIAL_ECHO_F(abs(diff),3);
      SERIAL_ECHOPAIR_F('\t',working_seg->a[i].expectedPosition,2);
    }
    }*/

  SERIAL_ECHO('\t');
  // SERIAL_ECHO(TEST(sensorManager.positionErrorFlags,POSITION_ERROR_FLAG_CONTINUOUS)?'+':'-');
  SERIAL_ECHO(TEST(sensorManager.positionErrorFlags, POSITION_ERROR_FLAG_ERROR) ? '+' : '-');
  // SERIAL_ECHO(TEST(sensorManager.positionErrorFlags,POSITION_ERROR_FLAG_FIRSTERROR)?'+':'-');
  // SERIAL_ECHO(TEST(sensorManager.positionErrorFlags,POSITION_ERROR_FLAG_ESTOP)?'+':'-');
  SERIAL_EOL();
}
#endif

#if MACHINE_STYLE == SIXI
// D18 copy sensor values to motor step positions.
void Parser::D18() {
  planner.wait_for_empty_segment_buffer();

  float a[NUM_AXIES];
  int numSamples = 10;

  for (ALL_AXIES(j)) { a[j] = 0; }

  // assert(NUM_SENSORS <= NUM_AXIES);

  for (int i = 0; i < numSamples; ++i) {
    sensorManager.updateAll();
    for (ALL_SENSORS(j)) { a[j] += sensorManager.sensors[j].angle; }
  }
  for (ALL_AXIES(j)) { a[j] /= (float)numSamples; }

  Planner::teleport(a);
}

// D19 - Sixi only.  toggle continuous D17 reporting
void Parser::D19() {
  boolean p = TEST(sensorManager.positionErrorFlags, POSITION_ERROR_FLAG_CONTINUOUS);
  int state = parseInt('P', p ? 1 : 0);
  SET_BIT(sensorManager.positionErrorFlags, POSITION_ERROR_FLAG_CONTINUOUS, state);

  SERIAL_ECHO_PAIR(F("D19 P",state ? 1 : 0, DEC);
}

void Parser::D20() {
  SET_BIT_OFF(sensorManager.positionErrorFlags, POSITION_ERROR_FLAG_ERROR);
}

void Parser::D21() {
  int isOn = parseInt('P', TEST_LIMITS ? 1 : 0);

  SET_BIT(sensorManager.positionErrorFlags, POSITION_ERROR_FLAG_CHECKLIMIT, isOn);

  SERIAL_ECHO("D21 ",TEST_LIMITS ? "1" : "0");
}

// D23 - Sixi is at the calibration position.  Set the home position accordingly.
void Parser::D23() {
  SERIAL_ECHO("D23");
  
  // cancel the current home offsets
  sensorManager.resetAll();
  // read the sensor
  sensorManager.updateAll();
  // apply the new offsets
  float homePos[NUM_AXIES];
  for (ALL_SENSORS(i)) { homePos[i] = sensorManager.sensors[i].angle; }
#ifdef CALIBRATION_TOOL
  // subtract the calibration from this position
  homePos[0] += 0;
  homePos[1] += -41.3;
  homePos[2] += 74.5;
  homePos[3] += 0;
  homePos[4] += -33.5;
  homePos[5] += 0;
#else
  // subtract the calibration from this position
  homePos[0] += 0;
  homePos[1] += -90;
  homePos[2] += 0;
  homePos[3] += 0;
  homePos[4] += 0;
  homePos[5] += 0;
#endif

  setHome(homePos);
}
#endif

// D50 Snn - Set and report strict mode.  where nn=0 for off and 1 for on.
void Parser::D50() {
  int oldValue = IS_STRICT;
  int newValue = parseInt('S', oldValue);
  SET_BIT(parserFlags, FLAG_STRICT, newValue);
  SERIAL_ECHOPAIR("D50 S",newValue ? 1 : 0);
}

// G commands

/**
   G0-G1 [Xnnn] [Ynnn] [Znnn] [Unnn] [Vnnn] [Wnnn] [Ann] [Fnn]
   straight lines.  distance in mm.
*/
void Parser::G01() {
#if MACHINE_STYLE == SIXI
  // if limit testing is on
  if(TEST_LIMITS) {
    // and a limit is exceeeded
    if(TEST(sensorManager.positionErrorFlags, POSITION_ERROR_FLAG_ERROR)) {
      // refuse to move
      SERIAL_ECHOLNPGM("LIMIT ERROR"));
      return;
    }
  }
#endif  // MACHINE_STYLE == SIXI

#ifdef HAS_GRIPPER
  if(hasGCode('T')>=0) {
    float toolStatus = parseFloat('T', 0);
    gripperUpdate(toolStatus);
  }
#endif

  int8_t offset=hasGCode('A');
  if(offset>=0) {
    desiredAcceleration = parseFloat(currentCommand+offset+1);
    desiredAcceleration = _MIN(_MAX(desiredAcceleration, (float)MIN_ACCELERATION), (float)MAX_ACCELERATION);
  }

  float f;
  offset=hasGCode('F');
  if(offset>=0) {
    f = parseFloat(currentCommand+offset+1);
    f = _MIN(_MAX(f, (float)MIN_FEEDRATE), (float)MAX_FEEDRATE);
  } else f = desiredFeedRate;

  bool bad=false;

  float pos[NUM_AXIES];
  for(ALL_AXIES(i)) {
    float p = axies[i].pos;
    offset=hasGCode(GET_AXIS_NAME(i));
    if(offset>=0) {
      float g = parseFloat(currentCommand+offset+1);
      if(absolute_mode) p = g;
      else p += g;

      if(p > axies[i].limitMax) {
        SERIAL_ECHOPAIR("LIMIT MAX ",i);
        SERIAL_ECHOPAIR_F(" | pos[i] = ",p);
        SERIAL_ECHOPAIR_F(" | axies[i].limitMax = ",axies[i].limitMax);
        bad=true;
      } else if(p < axies[i].limitMin) {
        SERIAL_ECHOPAIR("LIMIT MIN ",i);
        SERIAL_ECHOPAIR_F(" | pos[i] = ",p);
        SERIAL_ECHOPAIR_F(" | axies[i].limitMin = ",axies[i].limitMin);
        bad=true;
      }
    }
    pos[i]=p;
  }

  if(bad) return;

  planner.bufferLine(pos, f);
  printOK();
}

void Parser::printOK() {
  SERIAL_ECHOPAIR("OK N",lineNumber);
  SERIAL_ECHOPAIR(" P",Planner::movesFree());
  SERIAL_ECHOLNPAIR(" B",ringBuffer.spaceFree());
}

/**
   G2-G3 [Xnnn] [Ynnn] [Ann] [Fnn] [Inn] [Jnn]
   arcs in the XY plane
   @param clockwise (G2) 1 for cw, (G3) 0 for ccw
*/
void Parser::G02(int8_t clockwise) {
  desiredAcceleration = parseFloat('A', desiredAcceleration);
  desiredAcceleration = _MIN(_MAX(desiredAcceleration, (float)MIN_ACCELERATION), (float)MAX_ACCELERATION);
  float f = parseFloat('F', desiredFeedRate);
  f       = _MIN(_MAX(f, (float)MIN_FEEDRATE), (float)MAX_FEEDRATE);

  int i;
  float pos[NUM_AXIES];
  for (i = 0; i < NUM_AXIES; ++i) {
    float p = axies[i].pos;
    pos[i]  = parseFloat(GET_AXIS_NAME(i), (absolute_mode ? p : 0)) + (absolute_mode ? 0 : p);
  }

  float p0 = axies[0].pos;
  float p1 = axies[1].pos;
  planner.bufferArc(
    parseFloat('I', (absolute_mode ? p0 : 0)) + (absolute_mode ? 0 : p0),
    parseFloat('J', (absolute_mode ? p1 : 0)) + (absolute_mode ? 0 : p1),
    pos,
    clockwise,
    f
  );
  printOK();
}

/**
   G4 [Snn] [Pnn]
   Wait S milliseconds and P seconds.
*/
void Parser::G04() {
  planner.wait_for_empty_segment_buffer();
  uint32_t delayTime = parseInt('S', 0) + parseFloat('P', 0) * 1000.0f;
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
    pos[i]  = parseFloat(GET_AXIS_NAME(i), (absolute_mode ? p : 0)) + (absolute_mode ? 0 : p);
  }
  Planner::teleport(pos);
}

// M commands

/**
   M6 [Tnnn]
   Change the currently active tool
*/
void Parser::M6() {
  int tool_id = parseInt('T', current_tool);
  if(tool_id < 0) tool_id = 0;
  if(tool_id >= NUM_TOOLS) tool_id = NUM_TOOLS - 1;
  current_tool = tool_id;
}


void Parser::M17() {
  SERIAL_ECHOPGM("M17");

  motor.engage();
}


void Parser::M18() {
  SERIAL_ECHOPGM("M18");
  
  motor.disengage();
}


/**
   M42 P[a] S[b]
   Set digital pin a to state b (1 or 0).
   default pin is LED_BUILTIN.  default state is LOW
*/
void Parser::M42() {
  int pin      = parseInt('P', LED_BUILTIN);
  int newState = parseInt('S', 0);
  digitalWrite(pin, newState ? HIGH : LOW);
}

/**
 * M92 [Xn] [Yn] [Zn] [Un] [Vn] [Wn] - Set steps per unit
 */
void Parser::M92() {
  SERIAL_ECHOPGM("M92");
  for(ALL_MUSCLES(i)) {
    motor_spu[i] = parseFloat(motors[i].letter, motor_spu[i]);
    SERIAL_CHAR(' ');
    SERIAL_CHAR(motors[i].letter);
    SERIAL_ECHO_F(motor_spu[i]);
  }
  SERIAL_EOL();
}

void Parser::sayBuildDateAndTime() {
  SERIAL_ECHOLNPAIR("Built ",__DATE__ " " __TIME__);
}

void Parser::sayModelAndUID() {
  SERIAL_ECHOPAIR("I AM ",MACHINE_STYLE_NAME);
  SERIAL_ECHOLNPAIR(" #",robot_uid);
}

/**
   M100
   Print a M100ful message to MYSERIAL1.  The first line must never be changed to play nice with the JAVA software.
*/
void Parser::M100() {
  SERIAL_ECHOPGM("\n\nHELLO WORLD! ");
  sayModelAndUID();
  // report firmware version
  D5();
  sayBuildDateAndTime();
  SERIAL_ECHOLNPGM("Please see http://makelangelo.com/ for more information.");
  SERIAL_ECHOLNPGM("Try these (with a newline): G00,G01,G02,G03,G04,G28,G90,G91,G92,M18,M101,M100,M114");
#ifdef HAS_WIFI
  // Print the IP address
  SERIAL_ECHOPGM("Use this URL to connect: http://");
  SERIAL_ECHO(WiFi.softAPIP());
  SERIAL_ECHOPGM(':');
  SERIAL_ECHO(localPort);
  SERIAL_ECHOLNPGM('/');
#endif  // HAS_WIFI
#ifdef HAS_LCD
  SERIAL_ECHOPGM("Has LCD");
#endif
}

/**
   M101 Annn Tnnn Bnnn
   Change axis A limits to max T and min B.
   look for change to dimensions in command, apply and save changes.
*/
void Parser::M101() {
  int axisNumber = parseInt('A', -1);
  if(axisNumber >= 0 && axisNumber < NUM_AXIES) {
    float newT      = parseFloat('T', axies[axisNumber].limitMax);
    float newB      = parseFloat('B', axies[axisNumber].limitMin);
    uint8_t changed = 0;

    if(!equalEpsilon(axies[axisNumber].limitMax, newT)) {
      axies[axisNumber].limitMax = newT;
      changed                    = 1;
    }
    if(!equalEpsilon(axies[axisNumber].limitMin, newB)) {
      axies[axisNumber].limitMin = newB;
      changed                    = 1;
    }
    if(changed == 1) { eepromManager.saveLimits(); }
  }

  SERIAL_ECHOPGM("M101 (");

  for (ALL_AXIES(i)) {
    SERIAL_ECHO_F(axies[i].limitMin);
    if(i < NUM_AXIES - 1) SERIAL_CHAR(',');
  }

  SERIAL_ECHOPGM(") - (");

  for (ALL_AXIES(i)) {
    SERIAL_ECHO_F(axies[i].limitMax);
    if(i < NUM_AXIES - 1) SERIAL_CHAR(',');
  }

  SERIAL_ECHOLNPGM(")");
}

void Parser::M110() {
  lineNumber = parseInt('N', lineNumber);
}

/**
 * M112
 * Emergency stop
 */
void Parser::M112() {
  planner.estop();
#ifdef HAS_LCD
  LCD_setStatusMessage("ESTOP - PLEASE RESET");
#endif
  // stop clock
  DISABLE_ISRS();
  // do nothing
}

/**
 * M114
 * Print the X,Y,Z, feedrate, acceleration, and home position
 */
void Parser::M114() {
  planner.wait_for_empty_segment_buffer();

  SERIAL_ECHOPGM("M114");

  for(ALL_AXIES(i)) {
    SERIAL_CHAR(' ');
    SERIAL_CHAR(GET_AXIS_NAME(i));
    SERIAL_ECHO_F(axies[i].pos);
  }

  SERIAL_ECHOPAIR_F(" F",desiredFeedRate);
  SERIAL_ECHOPAIR_F(" A",desiredAcceleration);

  for(ALL_AXIES(i)) {
    SERIAL_CHAR(' ');
    SERIAL_CHAR(GET_AXIS_NAME(i));
    SERIAL_ECHO(Stepper::count_position[i]);
  }

  SERIAL_EOL();
}

#ifdef HAS_LCD
/**
   M117 [string]
   Display string on the LCD panel.  Command is ignored if there is no LCD panel.
*/
void Parser::M117() {
  // find M
  uint16_t i = 0;
  // skip "N*** M117 "
  while (currentCommand[i] != 'M') ++i;
  i += 5;
  // anything left?
  if(i >= strlen(currentCommand)) {
    // no message
    LCD_setStatusMessage(0);
    return;
  }

  // read in any remaining message
  char message[M117_MAX_LEN];
  char *buf = currentCommand + i;
  i         = 0;
  while (isPrintable(*buf) && (*buf) != '\r' && (*buf) != '\n' && i < M117_MAX_LEN) {
    message[i++] = *buf;
    buf++;
  }

  // make sure the message is properly terminated
  message[i] = 0;
  // update the LCD.
  LCD_setStatusMessage(message);
}
#endif  // HAS_LCD

/**
   M203 [Xnn] [Ynn] [Znn] [Unn] [Vnn] [Wnn]
   adjust the max feedrate of each axis
*/
void Parser::M203() {
  SERIAL_ECHOPGM("M203 ");
  for(ALL_MUSCLES(i)) {
    max_step_rate[i] = parseFloat(motors[i].letter, max_step_rate[i]);
    SERIAL_CHAR(motors[i].letter);
    SERIAL_ECHO_F(max_step_rate[i]);
    SERIAL_CHAR(' ');
  }
  SERIAL_EOL();
}

/**
   M205 X<jerk> Y<jerk> Z<jerk> U<jerk> V<jerk> W<jerk> B<us>
   adjust max jerk for axies XYZUVW.
   Adjust minimum segment time B
*/
void Parser::M205() {
  float f;
  SERIAL_ECHOPGM("M205");

#ifdef HAS_JUNCTION_DEVIATION
  f = parseFloat('J',Planner::junction_deviation);
  Planner::junction_deviation = _MAX(_MIN(f, (float)JUNCTION_DEVIATION_MAX), (float)JUNCTION_DEVIATION_MIN);
  SERIAL_ECHOPGM(" J");
  SERIAL_ECHO_F(Planner::junction_deviation);
#endif

#define PARSE_205_AXIS(AA,BB) \
  f = parseFloat(AA, max_jerk[BB]); \
  max_jerk[BB] = _MAX(_MIN(f, (float)MAX_JERK), (float)0); \
  SERIAL_CHAR(' '); \
  SERIAL_CHAR(AA); \
  SERIAL_ECHO_F(max_jerk[BB]);

  PARSE_205_AXIS('X',0);
#if NUM_AXIES > 1
  PARSE_205_AXIS('Y',1);
#endif
#if NUM_AXIES > 2
  PARSE_205_AXIS('Z',2);
#endif
#if NUM_AXIES > 3
  PARSE_205_AXIS('U',3);
#endif
#if NUM_AXIES > 4
  PARSE_205_AXIS('V',4);
#endif
#if NUM_AXIES > 5
  PARSE_205_AXIS('W',5);
#endif
  int16_t i = parseInt('B', Stepper::min_segment_time_us);
  Stepper::min_segment_time_us = _MAX(_MIN(i, 30000), 0);
  SERIAL_ECHOLNPAIR(" B",Stepper::min_segment_time_us);
}

/**
   M226 P[a] S[b]
   Wait for pin a to be in state b (1 or 0).
   If there is an LCD and P or S are missing, wait for user to press click wheel on LCD.
   If there is no LCD, P must be specified.
*/
void Parser::M226() {
#ifdef HAS_LCD
  int pin = parseInt('P', BTN_ENC);
#else
  int pin = parseInt('P', -1);
#endif
  if(pin == -1) return;  // no pin specified.

  int oldState = parseInt('S', -1);
  if(oldState == -1) {
    // default: assume the pin is not in the requested state
    oldState = digitalRead(pin);
  } else {
    // 0 for HIGH, anything else for LOW
    oldState = (oldState == 0) ? HIGH : LOW;
  }
  SERIAL_ECHOPGM("pausing");
#ifdef HAS_SD
  sd.sd_printing_paused = true;
#endif

  // while pin is in oldState (opposite of state for which we are waiting)
  while (digitalRead(pin) == oldState) {
    meanwhile();
#ifdef HAS_SD
    // SD_check();
#endif
#ifdef HAS_LCD
    // LCD_update();  // causes menu to change when we don't want it to change.
#endif
    //SERIAL_CHAR('.');
  }

#ifdef HAS_SD
  sd.sd_printing_paused = false;
#endif

  SERIAL_ECHOPGM(" ended.");
}

// M270 P[a] S[b] - Move servo P to angle S (1..180 for degrees, >=500 for pwm).
void Parser::M280() {
#if NUM_SERVOS > 0
  SERIAL_ECHOPGM("M280");
  
  Planner::wait_for_empty_segment_buffer();

  int id = parseInt('P', 0);
  int v = parseInt('S', servo[0].read());
  if(id <0 || id>=NUM_SERVOS) return;
  if(v>=0 && v <=180) {
    MOVE_SERVO(id,v);
  } else if(v>500) {
    servo[id].writeMicroseconds(v);
  }

  SERIAL_ECHOPAIR(" P",id);
  SERIAL_ECHOLNPAIR(" S",v);
#endif
}

/**
   M300 S[a] P[b]
   play frequency a for b milliseconds
*/
void Parser::M300() {
#ifdef HAS_LCD
  int ms = parseInt('P', 250);
  // int freq = parseInt('S', 60);

  digitalWrite(BEEPER, HIGH);
  delay(ms);
  digitalWrite(BEEPER, LOW);
#endif
}

#if MACHINE_STYLE == SIXI

// M428 - set home position to the current angle values
void Parser::M428() {
  // cancel the current home offsets
  sensorManager.resetAll();

  // read the sensor
  sensorManager.updateAll();

  // apply the new offsets
  for (ALL_MOTORS(i)) { axies[i].homePos = sensorManager.sensors[i].angle; }
  D18();
}
#endif

// M500 - save settings
void Parser::M500() {
  eepromManager.saveAll();
}

// M501 - reload settings
void Parser::M501() {
  eepromManager.loadAll();
}

// M502 - factory reset
void Parser::M502() {
  factory_reset();
}

// M503 - report all settings
void Parser::M503() {
  eepromManager.reportAll();
}

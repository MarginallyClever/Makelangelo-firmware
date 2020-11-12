#pragma once

//------------------------------------------------------------------------------
// SERIAL
//------------------------------------------------------------------------------

// How fast is the Arduino talking?
#ifndef BAUD
#define BAUD                 (57600)
#endif

// What is the longest message Arduino can store?
#define MAX_BUF              (128)


// for arc directions
#define ARC_CW               (1)
#define ARC_CCW              (-1)


// state flags for the parser
#define FLAG_RELATIVE      (0)  // relative moves
#define FLAG_STRICT        (1)  // force CRC check on all commands
#define FLAG_ECHO          (2)

#define RELATIVE_MOVES     (TEST(parserFlags,FLAG_RELATIVE))
#define IS_STRICT          (TEST(parserFlags,FLAG_STRICT))
#define MUST_ECHO          (TEST(parserFlags,FLAG_ECHO))


class Parser {
public:
  char serialBuffer[MAX_BUF + 1]; // Serial buffer
  int sofar;                      // Serial buffer progress
  uint32_t lastCmdTimeMs;         // prevent timeouts
  int32_t lineNumber = 0;        // make sure commands arrive in order
  uint8_t lastGcommand = -1;
  
  uint8_t parserFlags = 0;

  void start();
  // does this command have the matching code?
  uint8_t hasGCode(char code);
  // find the matching code and return the number that immediately follows it.
  float parseNumber(char code, float val);
  // if there is a line number, checks it is the correct line number.  if there is a * at the end, checks the string is valid.
  char checkLineNumberAndCRCisOK();
  // signal through serial I am ready to receive more commands
  void ready();  
  // read any available serial data
  void update();
  // called by update when an entire command is received.
  void processCommand();

  void sayBuildDateAndTime();
  void sayModelAndUID();


  void D0();  // D0 [Lnn] [Rnn] [Unn] [Vnn] [Wnn] [Tnn] - Jog each motor nn steps.
  void D5();  // D5 - report current firmware version
  void D6();  // D6 [Xnn] [Ynn] [Znn] [Unn] [Vnn] [Wnn] - Set home position for each axis.
#if MACHINE_STYLE == POLARGRAPH
  void D7();  // D7 [Lnn] [Rnn] - Polargraph only.  Set calibration length of each belt (mm)
  void D8();  // D8 - Polargraph only.  Report calibration values for left and right belts (mm)
  //void D13();  // D13 Znn - Polargraph only.  Set pen angle
#endif
  void D14();  // D14 - get machine style
#if MACHINE_STYLE == SIXI
  void D15();  // D15 - Sixi only.  Sixi demo
  void D17();  // D17 - Sixi only.  report sensor values for each axis.
  void D18();  // D18 - Sixi only.  copy sensor values to motor step positions (set current position)
  void D19();  // D19 - Sixi only.  toggle continuous D17 reporting
  void D20();  // D20 - Sixi only.  clear error flags
  void D21();  // D21 - Sixi only.  toggle software ESTOP
  void D23();  // D23 - Sixi only.  Sixi is at the calibration point.  Set the home position accordingly.
#endif
  void D50();  // D50 Snn - Set and report strict mode.  where nn=0 for off and 1 for on.

  
  void G01();  // G0/G1 [Xn] [Yn] [Zn] [Un] [Vn] [Wn] - linear travel
  void G02(int8_t clockwise);  // G02/G03 arcs
  void G04();  // G04 [Snn] [Pnn] - Wait S milliseconds and P seconds.
  void G28();  // G28 - go home
  void G90();  // G90 - set absolute mode
  void G91();  // G91 - set relative mode
  void G92();  // G92 - teleport


  void M6();  // M6 Tnn - change tool to nn
  //M17();  // M17 - engage motors
  //M18();  // M18 - disengage motors
  //M20();  // M20 - list SD card contents
  void M42();  // M42 Paa Sbb - Set digital pin aa to state bb (1 or 0).
  void M100();  // M100 - Print a helpful message to serial.
  void M101();  // M101 Annn Tnnn Bnnn - Change axis A limits to max T and min B.
  void M110();  // M110 Nnn - sets next expected line number to n.
  //void M111();  // M111 - Snn - sets parser flags to n.  Combine any valid flags: 0 (relative) 1 (strict) 2 (echo)
  void M112();  // M112 - emergency stop.
  void M114();  // M114 - report current target position
#ifdef HAS_LCD
  void M117();  // M117 [string] - Display string on the LCD panel.
#endif
  void M203();  // M203 [Xnn] [Ynn] [Znn] [Unn] [Vnn] [Wnn] - set max feedrate per axis
  void M205();  // M205 [Xnn] [Ynn] [Znn] [Unn] [Vnn] [Wnn] [Bnn] - set jerk for all axies. B is minimum segment time (us)
  void M206();  // M206 [Xn] [Yn] [Zn] [Un] [Vn] [Wn] - set home offsets 
  void M226();  // M226 P[a] S[b] - Wait for pin P to be in state S (1 or 0).
  void M300();  // M300 S[a] P[b] - play frequency a for b milliseconds (if there is a speaker)
#if MACHINE_STYLE == SIXI
  //void M306();  // M306 L[0...5] [Pn] [In] [Dn] - Sixi only.  adjust PID and report new values.
  void M428();  // M428 - Sixi only.  set home position to the current raw angle values (don't use home position to adjust home position!)
#endif
  void M500();  // M500 - save settings
  void M501();  // M501 - reload settings
  void M502();  // M502 - factory reset
  void M503();  // M503 - report all EEPROM settings
};

extern Parser parser;

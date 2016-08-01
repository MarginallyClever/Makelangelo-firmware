//------------------------------------------------------------------------------
// Makelangelo - supports raprapdiscount RUMBA controller
// dan@marginallycelver.com 2013-12-26
// RUMBA should be treated like a MEGA 2560 Arduino.
//------------------------------------------------------------------------------
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/Makelangelo for more information.


//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#include "configure.h"

#include <SPI.h>  // pkm fix for Arduino 1.5

#include "Vector3.h"


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------

// robot UID
int robot_uid=0;

// plotter limits, relative to the center of the plotter.
float limit_top = 0;  // distance to top of drawing area.
float limit_bottom = 0;  // Distance to bottom of drawing area.
float limit_right = 0;  // Distance to right of drawing area.
float limit_left = 0;  // Distance to left of drawing area.

static float homeX=0;
static float homeY=0;

// what are the motors called?
char m1d='L';
char m2d='R';

// motor inversions
char m1i=1;
char m2i=1;

// calculate some numbers to help us find feed_rate
float pulleyDiameter = 4.0f/PI;  // cm
float threadPerStep=0;

// plotter position.
float posx, posy, posz;  // pen state
float feed_rate=DEFAULT_FEEDRATE;
float acceleration=DEFAULT_ACCELERATION;

char absolute_mode=1;  // absolute or incremental programming mode?

// Serial comm reception
char serialBuffer[MAX_BUF+1];  // Serial buffer
int sofar;               // Serial buffer progress
static long last_cmd_time;    // prevent timeouts


Vector3 tool_offset[NUM_TOOLS];
int current_tool=0;


long line_number=0;


extern long global_steps_0;
extern long global_steps_1;


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// calculate max velocity, threadperstep.
void adjustPulleyDiameter(float diameter1) {
  pulleyDiameter = diameter1;
  float circumference = pulleyDiameter*PI;  // circumference
  threadPerStep = circumference/STEPS_PER_TURN;  // thread per step
}


//------------------------------------------------------------------------------
// returns angle of dy/dx as a value from 0...2PI
float atan3(float dy,float dx) {
  float a=atan2(dy,dx);
  if(a<0) a=(PI*2.0)+a;
  return a;
}


//------------------------------------------------------------------------------
char readSwitches() {
#ifdef USE_LIMIT_SWITCH
  // get the current switch state
  return ( (analogRead(L_PIN) < SWITCH_HALF) | (analogRead(R_PIN) < SWITCH_HALF) );
#else
  return 0;
#endif  // USE_LIMIT_SWITCH
}


//------------------------------------------------------------------------------
// feed rate is given in units/min and converted to cm/s
void setFeedRate(float v1) {
  if( feed_rate != v1 ) {
    feed_rate = v1;
    if(feed_rate > MAX_FEEDRATE) feed_rate = MAX_FEEDRATE;
    if(feed_rate < MIN_FEEDRATE) feed_rate = MIN_FEEDRATE;
#ifdef VERBOSE
    Serial.print(F("F="));
    Serial.println(feed_rate);
#endif
  }
}


//------------------------------------------------------------------------------
// delay in microseconds
void pause(long us) {
  delay(us / 1000);
  delayMicroseconds(us % 1000);
}


//------------------------------------------------------------------------------
void printFeedRate() {
  Serial.print(F("F"));
  Serial.print(feed_rate);
  Serial.print(F("steps/s"));
}


//------------------------------------------------------------------------------
// Inverse Kinematics - turns XY coordinates into lengths L1,L2
void IK(float x, float y, long &l1, long &l2) {
#ifdef COREXY
  l1 = lround((x+y) / threadPerStep);
  l2 = lround((x-y) / threadPerStep);
#endif
#ifdef TRADITIONALXY
  l1 = lround((x) / threadPerStep);
  l2 = lround((y) / threadPerStep);
#endif
#ifdef POLARGRAPH2
  // find length to M1
  float dy = y - limit_top;
  float dx = x - limit_left;
  l1 = lround( sqrt(dx*dx+dy*dy) / threadPerStep );
  // find length to M2
  dx = limit_right - x;
  l2 = lround( sqrt(dx*dx+dy*dy) / threadPerStep );
#endif
}


//------------------------------------------------------------------------------
// Forward Kinematics - turns L1,L2 lengths into XY coordinates
// use law of cosines: theta = acos((a*a+b*b-c*c)/(2*a*b));
// to find angle between M1M2 and M1P where P is the plotter position.
void FK(long l1, long l2,float &x,float &y) {
#ifdef COREXY
  l1 *= threadPerStep;
  l2 *= threadPerStep;

  x = (float)( l1 + l2 ) / 2.0;
  y = x - (float)l2;
#endif
#ifdef TRADITIONALXY
  x = l1 * threadPerStep;
  y = l2 * threadPerStep;
#endif
#ifdef POLARGRAPH2
  float a = (float)l1 * threadPerStep;
  float b = (limit_right-limit_left);
  float c = (float)l2 * threadPerStep;

  // slow, uses trig
  // we know law of cosines:   cc = aa + bb -2ab * cos( theta )
  // or cc - aa - bb = -2ab * cos( theta )
  // or ( aa + bb - cc ) / ( 2ab ) = cos( theta );
  // or theta = acos((aa+bb-cc)/(2ab));
  //x = cos(theta)*l1 + limit_left;
  //y = sin(theta)*l1 + limit_top;
  // and we know that cos(acos(i)) = i
  // and we know that sin(acos(i)) = sqrt(1-i*i)
  float theta = ((a*a+b*b-c*c)/(2.0*a*b));
  x = theta * a + limit_left;
  y = limit_top - (sqrt( 1.0 - theta * theta ) * a);
#endif
}


//------------------------------------------------------------------------------
void processConfig() {
  float newT = parseNumber('T',limit_top);
  float newB = parseNumber('B',limit_bottom);
  float newR = parseNumber('R',limit_right);
  float newL = parseNumber('L',limit_left);

  adjustDimensions(newT,newB,newR,newL);

  // programmatically swap motors
  char gg=parseNumber('G',m1d);
  char hh=parseNumber('H',m2d);

  // invert motor direction
  char i=parseNumber('I',0);
  char j=parseNumber('J',0);
  
  adjustInversions(i,j);
  
  // @TODO: check t>b, r>l ?
  
  printConfig();

  teleport(posx,posy);
}


//------------------------------------------------------------------------------
void adjustInversions(int m1,int m2) {
  if(m1>0) {
    motors[0].reel_in  = HIGH;
    motors[0].reel_out = LOW;
  } else if(m1<0) {
    motors[0].reel_in  = LOW;
    motors[0].reel_out = HIGH;
  }

  if(m2>0) {
    motors[1].reel_in  = HIGH;
    motors[1].reel_out = LOW;
  } else if(m2<0) {
    motors[1].reel_in  = LOW;
    motors[1].reel_out = HIGH;
  }

  if( m1!=m1i || m2 != m2i) {
    m1i=m1;
    m2i=m2;
    saveInversions();
  }
}


/**
 * Test that IK(FK(A))=A
 */
void test_kinematics() {
  long A,B,i;
  float C,D,x=0,y=0;

  for(i=0;i<3000;++i) {
    x = random(limit_right,limit_right)*0.1;
    y = random(limit_bottom,limit_top)*0.1;

    IK(x,y,A,B);
    FK(A,B,C,D);
    Serial.print(F("\tx="));  Serial.print(x);
    Serial.print(F("\ty="));  Serial.print(y);
    Serial.print(F("\tL="));  Serial.print(A);
    Serial.print(F("\tR="));  Serial.print(B);
    Serial.print(F("\tx'="));  Serial.print(C);
    Serial.print(F("\ty'="));  Serial.print(D);
    Serial.print(F("\tdx="));  Serial.print(C-x);
    Serial.print(F("\tdy="));  Serial.println(D-y);
  }
}

/**
 * Translate the XYZ through the IK to get the number of motor steps and move the motors.
 * @input x destination x value
 * @input y destination y value
 * @input z destination z value
 * @input new_feed_rate speed to travel along arc
 */
void polargraph_line(float x,float y,float z,float new_feed_rate) {
  long l1,l2;
  IK(x,y,l1,l2);
  posx=x;
  posy=y;
  posz=z;
/*
  Serial.print('~');
  Serial.print(x);  Serial.print('\t');
  Serial.print(y);  Serial.print('\t');
  Serial.print(z);  Serial.print('\t');
  Serial.print(l1);  Serial.print('\t');
  Serial.print(l2);  Serial.print('\n');
  */
  feed_rate = new_feed_rate;
  motor_line(l1,l2,z,new_feed_rate);
}


/**
 * Move the pen holder in a straight line using bresenham's algorithm
 * @input x destination x value
 * @input y destination y value
 * @input z destination z value
 * @input new_feed_rate speed to travel along arc
 */
void line_safe(float x,float y,float z,float new_feed_rate) {
  x-=tool_offset[current_tool].x;
  y-=tool_offset[current_tool].y;
  z-=tool_offset[current_tool].z;

  // split up long lines to make them straighter?
  Vector3 destination(x,y,z);
  Vector3 startPoint(posx,posy,posz);
  Vector3 dp = destination - startPoint;
  Vector3 temp;

  float len=dp.Length();
  int pieces = ceil(dp.Length() * (float)SEGMENT_PER_CM_LINE );

  float a;
  long j;

  // draw everything up to (but not including) the destination.
  for(j=1;j<pieces;++j) {
    a=(float)j/(float)pieces;
    temp = dp * a + startPoint;
    polargraph_line(temp.x,temp.y,temp.z,new_feed_rate);
  }
  // guarantee we stop exactly at the destination (no rounding errors).
  polargraph_line(x,y,z,new_feed_rate);
  /*
  long l1,l2;
  IK(x,y,l1,l2);
  Serial.print(F("H0="));  Serial.print(l1);
  Serial.print(F("\tH1="));  Serial.println(l2);
  */
}


/**
 * This method assumes the limits have already been checked.
 * This method assumes the start and end radius match.
 * This method assumes arcs are not >180 degrees (PI radians)
 * @input cx center of circle x value
 * @input cy center of circle y value
 * @input x destination x value
 * @input y destination y value
 * @input z destination z value
 * @input dir - ARC_CW or ARC_CCW to control direction of arc
 * @input new_feed_rate speed to travel along arc
 */
void arc(float cx,float cy,float x,float y,float z,char clockwise,float new_feed_rate) {
  // get radius
  float dx = posx - cx;
  float dy = posy - cy;
  float sr=sqrt(dx*dx+dy*dy);

  // find angle of arc (sweep)
  float sa=atan3(dy,dx);
  float ea=atan3(y-cy,x-cx);
  float er=sqrt(dx*dx+dy*dy);
  
  float da=ea-sa;
  if(clockwise!=0 && da<0) ea+=2*PI;
  else if(clockwise==0 && da>0) sa+=2*PI;
  da=ea-sa;
  float dr = er-sr;

  // get length of arc
  // float circ=PI*2.0*radius;
  // float len=theta*circ/(PI*2.0);
  // simplifies to
  float len1 = abs(da) * sr;
  float len = sqrt( len1 * len1 + dr * dr );

  int i, segments = ceil( len * SEGMENT_PER_CM_ARC );

  float nx, ny, nz, angle3, scale;
  float a,r;
  for(i=0;i<=segments;++i) {
    // interpolate around the arc
    scale = ((float)i)/((float)segments);

    a = ( da * scale ) + sa;
    r = ( dr * scale ) + sr;
    
    nx = cx + cos(a) * r;
    ny = cy + sin(a) * r;
    nz = ( z - posz ) * scale + posz;
    // send it to the planner
    line_safe(nx,ny,nz,new_feed_rate);
  }
}


/**
 * Instantly move the virtual plotter position.  Does not check if the move is valid.
 */
void teleport(float x,float y) {
  wait_for_empty_segment_buffer();
  
  posx=x;
  posy=y;

  // @TODO: posz?
  long L1,L2;
  IK(posx,posy,L1,L2);

  motor_set_step_count(L1,L2,0);
}


/**
 * Print a helpful message to serial.  The first line must never be changed to play nice with the JAVA software.
 */
void help() {
  Serial.print(F("\n\nHELLO WORLD! I AM DRAWBOT #"));
  Serial.println(robot_uid);
  sayVersionNumber();
  Serial.println(F("Model RUMBA"));
  Serial.println(F("== http://www.makelangelo.com/ =="));
  Serial.println(F("M100 - display this message"));
  Serial.println(F("M101 [Tx.xx] [Bx.xx] [Rx.xx] [Lx.xx]"));
  Serial.println(F("       - display/update board dimensions."));
  Serial.println(F("As well as the following G-codes (http://en.wikipedia.org/wiki/G-code):"));
  Serial.println(F("G00,G01,G02,G03,G04,G28,G90,G91,G92,M18,M114"));
}


void sayVersionNumber() {
  char versionNumber = loadVersion();
  
  Serial.print(F("Firmware v"));
  Serial.print(versionNumber,DEC);
}


/**
 * if limit switches are installed, move to touch each switch so that the pen holder can move to home position.
 */
void findHome() {
#ifdef USE_LIMIT_SWITCH
  Serial.println(F("Homing..."));

  if(readSwitches()) {
    Serial.println(F("** ERROR **"));
    Serial.println(F("Problem: Plotter is already touching switches."));
    Serial.println(F("Solution: Please unwind the strings a bit and try again."));
    return;
  }

  int safe_out=50;

  // reel in the left motor until contact is made.
  Serial.println(F("Find left..."));
  digitalWrite(motors[0].dir_pin,HIGH);
  digitalWrite(motors[1].dir_pin,LOW);
  do {
    digitalWrite(motors[0].step_pin,HIGH);
    digitalWrite(motors[0].step_pin,LOW);
    digitalWrite(motors[1].step_pin,HIGH);
    digitalWrite(motors[1].step_pin,LOW);
    pause(STEP_DELAY);
  } while(!readSwitches());

  // back off so we don't get a false positive on the next motor
  int i;
  digitalWrite(motors[0].dir_pin,LOW);
  for(i=0;i<safe_out;++i) {
    digitalWrite(motors[0].step_pin,HIGH);
    digitalWrite(motors[0].step_pin,LOW);
    pause(STEP_DELAY);
  }

  // reel in the right motor until contact is made
  Serial.println(F("Find right..."));
  digitalWrite(motors[0].dir_pin,LOW);
  digitalWrite(motors[1].dir_pin,HIGH);
  do {
    digitalWrite(motors[0].step_pin,HIGH);
    digitalWrite(motors[0].step_pin,LOW);
    digitalWrite(motors[1].step_pin,HIGH);
    digitalWrite(motors[1].step_pin,LOW);
    pause(STEP_DELAY);
    laststep1++;
  } while(!readSwitches());

  // back off so we don't get a false positive that kills line()
  digitalWrite(motors[1].dir_pin,LOW);
  for(i=0;i<safe_out;++i) {
    digitalWrite(motors[1].step_pin,HIGH);
    digitalWrite(motors[1].step_pin,LOW);
    pause(STEP_DELAY);
  }

  Serial.println(F("Centering..."));
  polargraph_line(0,0,posz);
#endif // USE_LIMIT_SWITCH
}


/**
 * Print the X,Y,Z, feedrate, and acceleration to serial.
 * Equivalent to gcode M114
 */
void where() {
  wait_for_empty_segment_buffer();

  Serial.print(F("X"));   Serial.print(posx);
  Serial.print(F(" Y"));  Serial.print(posy);
  Serial.print(F(" Z"));  Serial.print(posz);
  Serial.print(' ');  printFeedRate();
  Serial.print(F(" A"));  Serial.println(acceleration);
  
  Serial.print(F(" HX="));  Serial.print(homeX);
  Serial.print(F(" HY="));  Serial.println(homeY);
  
  //Serial.print(F(" G0="));  Serial.print(global_steps_0);
  //Serial.print(F(" G1="));  Serial.print(global_steps_1);
}


/**
 * Print the machine limits to serial.
 */
void printConfig() {
  Serial.print(limit_left);       Serial.print(F(","));
  Serial.print(limit_top);        Serial.print(F(" - "));
  Serial.print(limit_right);     Serial.print(F(","));
  Serial.print(limit_bottom);      Serial.print(F("\n"));
}


/**
 * Set the relative tool offset
 * @input axis the active tool id
 * @input x the x offset
 * @input y the y offset
 * @input z the z offset
 */
void set_tool_offset(int axis,float x,float y,float z) {
  tool_offset[axis].x=x;
  tool_offset[axis].y=y;
  tool_offset[axis].z=z;
}


/**
 * @return the position + active tool offset
 */
Vector3 get_end_plus_offset() {
  return Vector3(tool_offset[current_tool].x + posx,
                 tool_offset[current_tool].y + posy,
                 tool_offset[current_tool].z + posz);
}


/**
 * Change the currently active tool
 */
void tool_change(int tool_id) {
  if(tool_id < 0) tool_id=0;
  if(tool_id >= NUM_TOOLS) tool_id=NUM_TOOLS-1;
  current_tool=tool_id;
}


/**
 * Look for character /code/ in the buffer and read the float that immediately follows it.
 * @return the value found.  If nothing is found, /val/ is returned.
 * @input code the character to look for.
 * @input val the return value if /code/ is not found.
 **/
float parseNumber(char code,float val) {
  char *ptr=serialBuffer;  // start at the beginning of buffer
  while(ptr && *ptr && ptr<serialBuffer+sofar) {  // walk to the end
    if(*ptr==code) {  // if you find code on your walk,
      return atof(ptr+1);  // convert the digits that follow into a float and return it
    }
    ptr=strchr(ptr,' ')+1;  // take a step from here to the letter after the next space
  }
  return val;  // end reached, nothing found, return default val.
}


/**
 * process commands in the serial receive buffer
 */
void processCommand() {
  // blank lines
  if(serialBuffer[0]==';') return;

  long cmd;

  // is there a line number?
  cmd=parseNumber('N',-1);
  if(cmd!=-1 && serialBuffer[0]=='N') {  // line number must appear first on the line
    if( cmd != line_number ) {
      // wrong line number error
      Serial.print(F("BADLINENUM "));
      Serial.println(line_number);
      return;
    }

    // is there a checksum?
    if(strchr(serialBuffer,'*')!=0) {
      // yes.  is it valid?
      char checksum=0;
      int c=0;
      while(serialBuffer[c]!='*' && c<MAX_BUF) checksum ^= serialBuffer[c++];
      c++; // skip *
      int against = strtod(serialBuffer+c,NULL);
      if( checksum != against ) {
        Serial.print(F("BADCHECKSUM "));
        Serial.println(line_number);
        return;
      }
    } else {
      Serial.print(F("NOCHECKSUM "));
      Serial.println(line_number);
      return;
    }

    line_number++;
  }

  if(!strncmp(serialBuffer,"UID",3)) {
    robot_uid=atoi(strchr(serialBuffer,' ')+1);
    saveUID();
  }


  cmd=parseNumber('M',-1);
  switch(cmd) {
  case 6:  tool_change(parseNumber('T',current_tool));  break;
  case 17:  motor_engage();  break;
  case 18:  motor_disengage();  break;
  case 100:  help();  break;
  case 101:  processConfig();  break;
  case 110:  line_number = parseNumber('N',line_number);  break;
  case 114:  where();  break;
  }

  cmd=parseNumber('G',-1);
  switch(cmd) {
  case 0:
  case 1: {  // line
      Vector3 offset=get_end_plus_offset();
      acceleration = min(max(parseNumber('A',acceleration),1),2000);
      line_safe( parseNumber('X',(absolute_mode?offset.x:0)*10)*0.1 + (absolute_mode?0:offset.x),
                 parseNumber('Y',(absolute_mode?offset.y:0)*10)*0.1 + (absolute_mode?0:offset.y),
                 parseNumber('Z',(absolute_mode?offset.z:0)) + (absolute_mode?0:offset.z),
                 parseNumber('F',feed_rate) );
      break;
    }
  case 2:
  case 3: {  // arc
      Vector3 offset=get_end_plus_offset();
      acceleration = min(max(parseNumber('A',acceleration),1),2000);
      setFeedRate(parseNumber('F',feed_rate));
      arc(parseNumber('I',(absolute_mode?offset.x:0)*10)*0.1 + (absolute_mode?0:offset.x),
          parseNumber('J',(absolute_mode?offset.y:0)*10)*0.1 + (absolute_mode?0:offset.y),
          parseNumber('X',(absolute_mode?offset.x:0)*10)*0.1 + (absolute_mode?0:offset.x),
          parseNumber('Y',(absolute_mode?offset.y:0)*10)*0.1 + (absolute_mode?0:offset.y),
          parseNumber('Z',(absolute_mode?offset.z:0)) + (absolute_mode?0:offset.z),
          (cmd==2) ? 1 : 0,
          parseNumber('F',feed_rate) );
      break;
    }
  case 4:  {  // dwell
      wait_for_empty_segment_buffer();
      float delayTime = parseNumber('S',0) + parseNumber('P',0)*1000.0f;
      pause(delayTime);
      break;
    }
  case 28:  findHome();  break;
  case 54:
  case 55:
  case 56:
  case 57:
  case 58:
  case 59: {  // 54-59 tool offsets
    int tool_id=cmd-54;
    set_tool_offset(tool_id,parseNumber('X',tool_offset[tool_id].x),
                            parseNumber('Y',tool_offset[tool_id].y),
                            parseNumber('Z',tool_offset[tool_id].z));
    break;
    }
  case 90:  absolute_mode=1;  break;  // absolute mode
  case 91:  absolute_mode=0;  break;  // relative mode
  case 92: {  // set position (teleport)
      Vector3 offset = get_end_plus_offset();
      teleport( parseNumber('X',(absolute_mode?offset.x:0)*10)*0.1 + (absolute_mode?0:offset.x),
                 parseNumber('Y',(absolute_mode?offset.y:0)*10)*0.1 + (absolute_mode?0:offset.y)
               //parseNumber('Z',(absolute_mode?offset.z:0)) + (absolute_mode?0:offset.z)
                 );
      break;
    }
  }

  cmd=parseNumber('D',-1);
  switch(cmd) {
  case 0: {
      // move one motor
      int i,amount=parseNumber(m1d,0);
      digitalWrite(MOTOR_0_DIR_PIN,amount < 0 ? motors[0].reel_in : motors[0].reel_out);
      amount=abs(amount);
      for(i=0;i<amount;++i) {
        digitalWrite(MOTOR_0_STEP_PIN,HIGH);
        digitalWrite(MOTOR_0_STEP_PIN,LOW);
        pause(STEP_DELAY);
      }

      amount=parseNumber(m2d,0);
      digitalWrite(MOTOR_1_DIR_PIN,amount < 0 ? motors[1].reel_in : motors[1].reel_out);
      amount = abs(amount);
      for(i=0;i<amount;++i) {
        digitalWrite(MOTOR_1_STEP_PIN,HIGH);
        digitalWrite(MOTOR_1_STEP_PIN,LOW);
        pause(STEP_DELAY);
      }
    }
    break;
  case 1: {
      // adjust spool diameters
      float amountL=parseNumber('L',pulleyDiameter);

      float d1=pulleyDiameter;
      adjustPulleyDiameter(amountL);
      if(pulleyDiameter != d1) {
        // Update EEPROM
        savePulleyDiameter();
      }
    }
    break;
  case 2:
    Serial.print('L');  Serial.print(pulleyDiameter);
    Serial.print(F(" R"));   Serial.println(pulleyDiameter);
    break;
//  case 3:  SD_ListFiles();  break;
  case 4:  SD_StartPrintingFile(strchr(serialBuffer,' ')+1);  break;  // read file
  case 5:
    sayVersionNumber();
  case 6:  // set home
    setHome(parseNumber('X',(absolute_mode?homeX:0)*10)*0.1 + (absolute_mode?0:homeX),
            parseNumber('Y',(absolute_mode?homeY:0)*10)*0.1 + (absolute_mode?0:homeY));
    break;
  }
}


void setHome(float x,float y) {
  if(x != homeX || y!=homeY) {
    homeX = x;
    homeY = y;
    saveHome();
  }
}


/**
 * prepares the input buffer to receive a new message and tells the serial connected device it is ready for more.
 */
void parser_ready() {
  sofar=0;  // clear input buffer
  Serial.print(F("\n> "));  // signal ready to receive input
  last_cmd_time = millis();
}


//------------------------------------------------------------------------------
void tools_setup() {
  for(int i=0;i<NUM_TOOLS;++i) {
    set_tool_offset(i,0,0,0);
  }
}


//------------------------------------------------------------------------------
void setup() {
  loadConfig();

  // start communications
  Serial.begin(BAUD);

  // display the help at startup.
  help();

  motor_setup();
  motor_engage();
  tools_setup();

  //easyPWM_init();
  SD_init();
  LCD_init();

  // initialize the plotter position.
  teleport(0,0);
  setPenAngle(PEN_UP_ANGLE);
  setFeedRate(DEFAULT_FEEDRATE);
  parser_ready();
}


//------------------------------------------------------------------------------
// See: http://www.marginallyclever.com/2011/10/controlling-your-arduino-through-the-serial-monitor/
void Serial_listen() {
  // listen for serial commands
  while(Serial.available() > 0) {
    char c = Serial.read();
    if(c=='\r') continue;
    if(sofar<MAX_BUF) serialBuffer[sofar++]=c;
    if(c=='\n') {
      serialBuffer[sofar]=0;

      // echo confirmation
//      Serial.println(F(serialBuffer));

      // do something with the command
      processCommand();
      parser_ready();
    }
  }
}


//------------------------------------------------------------------------------
void loop() {
  Serial_listen();
  SD_check();
#ifdef HAS_LCD
  LCD_update();
#endif

  // The PC will wait forever for the ready signal.
  // if Arduino hasn't received a new instruction in a while, send ready() again
  // just in case USB garbled ready and each half is waiting on the other.
  if( !segment_buffer_full() && (millis() - last_cmd_time) > TIMEOUT_OK ) {
    parser_ready();
  }
}


/**
 * This file is part of makelangelo-firmware.
 *
 * makelangelo-firmware is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * makelangelo-firmware is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DrawbotGUI.  If not, see <http://www.gnu.org/licenses/>.
 */

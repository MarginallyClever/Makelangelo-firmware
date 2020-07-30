//------------------------------------------------------------------------------
// HAND-E gripper module for Sixi Robot
// 2020-03-28 dan@marginallyclever.com
// CC-by-NC-SA
//------------------------------------------------------------------------------
#include "configure.h"

#ifdef HAS_GRIPPER

#include "gripper_hande.h"
#include <SoftwareSerial.h>

// DEFINES

//#ifdef DEBUG_GRIPPER

#define GRIPPER_BAUD      115200
#define GRIPPER_SLAVE_ID  9

// REGISTERS
// 16 one-byte registers
#define GRIPPER_FIRST_INPUT_REGISTER   0x07D0 // 2000
#define GRIPPER_FIRST_OUTPUT_REGISTER  0x03E8 // 1000

// Send registers
#define GRIPPER_REG_ACTION_REQUEST     0
  // action request bytes.  high to activate.
  #define GRIPPER_BIT_rACT             0  // enable pin?  Reset to clear fault status.
  #define GRIPPER_BIT_rGTO             3  // go to desired position.
  #define GRIPPER_BIT_rATR             4  // automatic release - slow opening of all grippers to maximum.  Emergency auto-release.
  #define GRIPPER_BIT_rARD             5  // auto release direction.  high for open, low for close.

#define GRIPPER_REG_POSITION_REQUEST   2  // 0...255.  0 is fully open, 255 is fully closed. 0.2mm per bit, 50mm range.  set this before setting rGTO.
#define GRIPPER_REG_SPEED              4  // 0...255.  Gripper speed.  255 is max.  set this before setting rGTO. 
#define GRIPPER_REG_FORCE              5  // 0...255.  Gripper force.  255 is max.  set this before setting rGTO.

// which gives us a total of...
#define GRIPPER_MAX_INPUT_BYTES        6
#define GRIPPER_MAX_INPUT_REGISTERS    (GRIPPER_MAX_INPUT_BYTES/2)


// Recv registers
#define GRIPPER_REG_STATUS             0
  // activation status.
  #define GRIPPER_BIT_gACT             0
  // goto status.  0 for stopped (or performing activation / automatic release).
  #define GRIPPER_BIT_gGTO             3
  // 0x00 - Gripper is in reset (or automatic release) state. See Fault Status if gripper is activated.
  // 0x01 - Activation in progress.
  // 0x02 - Not used.
  // 0x03 - Activation is completed.
  #define GRIPPER_BIT_gSTA1            4 
  #define GRIPPER_BIT_gSTA2            5
  // 0x00 - Fingers are in motion towards requested position. No object detected.
  // 0x01 - Fingers have stopped due to a contact while opening before requested position. Object detected opening.
  // 0x02 - Fingers have stopped due to a contact while closing before requested position. Object detected closing.
  // 0x03 - Fingers are at requested position. No object detected or object has been loss / dropped.
  #define GRIPPER_BIT_gOBJ1            6
  #define GRIPPER_BIT_gOBJ2            7
  
#define GRIPPER_REG_FAULT_STATUS       2
  // 0x00 - No fault (solid blue LED)
  // # Priority faults (solid blue LED)
  // 0x05 - Action delayed; the activation (re-activation) must be completed prior to perform the action.
  // 0x07 - The activation bit must be set prior to performing the action.
  // # Minor faults (solid red LED)
  // x08 - Maximum operating temperature exceeded (≥ 85 °C internally); let cool down (below 80 °C).
  // 0x09 - No communication during at least 1 second.
  // # Major faults (LED blinking red/blue) - Reset is required (rising edge on activation bit (rACT) needed).
  // 0x0A - Under minimum operating voltage.
  // 0x0B - Automatic release in progress.
  // 0x0C - Internal fault, contact support@robotiq.com
  // 0x0D - Activation fault, verify that no interference or other error occurred.
  // 0x0E - Overcurrent triggered.
  // 0x0F - Automatic release completed.
  #define GRIPPER_BIT_gFLT0            0
  #define GRIPPER_BIT_gFLT1            1
  #define GRIPPER_BIT_gFLT2            2
  #define GRIPPER_BIT_gFLT3            3
  // per-gripper values.
  #define GRIPPER_BIT_kFLT0            4
  #define GRIPPER_BIT_kFLT1            5
  #define GRIPPER_BIT_kFLT2            6
  #define GRIPPER_BIT_kFLT3            7
  
#define GRIPPER_REG_POS_REQUEST_ECHO   3  // 0...255.  Last requested gripper position.  255 is fully closed.
#define GRIPPER_REG_POSITION           4  // 0...255.  Gripper position.  255 is fully closed.
#define GRIPPER_REG_CURRENT            5  // 0...255.  Current from motor drive, where current = (10*current) mA.

// which gives us a total of...
#define GRIPPER_MAX_OUTPUT_BYTES       6
#define GRIPPER_MAX_OUTPUT_REGISTERS   (GRIPPER_MAX_OUTPUT_BYTES/2)


// must be inverted later!
#define GRIPPER_POS_MAX  255
#define GRIPPER_POS_MIN  0

#define GRIPPER_VEL_MAX  255
#define GRIPPER_VEL_MIN  0

#define GRIPPER_F_MAX    255
#define GRIPPER_F_MIN    0


// function messages
// Function code 04 (FC04) is used for requesting the status of the gripper analog input register. Examples of such data are gripper status, object status, finger position, etc.
// read values: slave id, 4, address of first read register, number of read registers, CRC
// response:    slave id, 4, address of first read register, number of read registers, [bytes of data], CRC
#define GRIPPER_FUNCTION_READ_REGISTER  4
// set values:  slave id, 16, address of first write register, number of write registers, [bytes of data], CRC
// response:    slave id, 16, address of first write register, number of write registers, CRC
#define GRIPPER_FUNCTION_SET_REGISTER   16
// set values:  slave id, 23, address of first read register, number of read registers, [bytes of data], address of first write register, number of write registers, [bytes of data], CRC
// response:    slave id, 23, address of first read register, number of read registers, [bytes of data], CRC
#define GRIPPER_FUNCTION_BOTH_REGISTER  23

// MACROS
#ifdef DEBUG_GRIPPER
#define PRINTHEX(x)          {  if(x<0x10) Serial.print('0');   Serial.print(x,HEX);  }
#define PRINT(x)              Serial.print(x)
#else
#define PRINTHEX(x)          
#define PRINT(x)             
#endif

#define WRITE_START          uint16_t crc=0xFFFF;  packed=0;
#define WRITE_LOW_NOCRC(x)   {  byte _b= lowByte(x);  outBuf[packed++]=_b;  PRINTHEX(_b);  PRINT(' ');  }
#define WRITE_HIGH_NOCRC(x)  {  byte _b=highByte(x);  outBuf[packed++]=_b;  PRINTHEX(_b);  PRINT(' ');  }
#define WRITE_LOW(x)         {  byte _b1= lowByte(x);  WRITE_LOW_NOCRC(x);  crc=crc16_update(crc,_b1);  }
#define WRITE_HIGH(x)        {  byte _b1=highByte(x);  WRITE_HIGH_NOCRC(x);  crc=crc16_update(crc,_b1);  }
#define WRITE_END            gripperSerial.write(outBuf,packed);


// GLOBALS
SoftwareSerial gripperSerial(GRIPPER_RX, GRIPPER_TX);
Gripper gripper;


uint16_t Gripper::crc16_update(uint16_t crc, uint8_t a) {
  int i;

  crc ^= a;
  for (i = 0; i < 8; ++i) {
    if (crc & 1) crc = (crc >> 1) ^ 0xA001;
    else         crc = (crc >> 1);
  }

  return crc;
}


void Gripper::setup() {
  Serial.begin(57600);
  gripperSerial.begin(GRIPPER_BAUD);
  sendActionRequest(0);
  delay(5);
  sendActionRequest(1);
  delay(5);
}


void Gripper::update() {
  readStatus();
}


/**
 * Go to position (0..255) with feedrate (0...255) and force (0...255)
 */
void Gripper::sendPositionRequest(uint8_t positionRequest,uint8_t speedRequest,uint8_t forceRequest) {
  PRINT("gripper pos >> ");

  WRITE_START;
  WRITE_LOW(GRIPPER_SLAVE_ID);
  WRITE_LOW(GRIPPER_FUNCTION_SET_REGISTER);
  
  WRITE_HIGH(GRIPPER_FIRST_OUTPUT_REGISTER);
  WRITE_LOW(GRIPPER_FIRST_OUTPUT_REGISTER);
  
  uint16_t registersTouched=3;
  WRITE_HIGH(registersTouched);
  WRITE_LOW(registersTouched);

  uint8_t bytesSent=6;
  WRITE_LOW(bytesSent);

  uint16_t command = (1<<GRIPPER_BIT_rACT)  // activate
                   | (1<<GRIPPER_BIT_rGTO);  // go to desired position
  
  WRITE_LOW(command);
  WRITE_HIGH(command);
  
  WRITE_LOW(00);
  WRITE_LOW(positionRequest);

  WRITE_LOW(speedRequest);
  WRITE_LOW(forceRequest);

  WRITE_LOW_NOCRC(crc);
  WRITE_HIGH_NOCRC(crc);
  WRITE_END;
  
  
  
  int bytesExpected = 1 // slave id
                    + 1 // function 16
                    + 2 // first register
                    + 2 // number of written registers
                    + 2 // crc
                    ;
  readResponse(bytesExpected);
}

void Gripper::sendActionRequest(uint8_t actionRequest) {
  PRINT("gripper action >> ");
  
  WRITE_START;
  WRITE_LOW(GRIPPER_SLAVE_ID);  // slave id (9)
  WRITE_LOW(GRIPPER_FUNCTION_SET_REGISTER);  // set register (16)
  
  WRITE_HIGH(GRIPPER_FIRST_OUTPUT_REGISTER);  // 2000
  WRITE_LOW(GRIPPER_FIRST_OUTPUT_REGISTER);  // 2000
  
  uint16_t registersTouched=3;
  WRITE_HIGH(registersTouched);
  WRITE_LOW(registersTouched);

  uint8_t bytesSent=registersTouched*2;
  WRITE_LOW(bytesSent);

  WRITE_LOW(actionRequest);
  WRITE_LOW(0);
  
  WRITE_LOW(0);
  WRITE_LOW(0);

  WRITE_LOW(0);
  WRITE_LOW(0);

  WRITE_LOW_NOCRC(crc);
  WRITE_HIGH_NOCRC(crc);
  WRITE_END;
  
  int bytesExpected = 1 // slave id
                    + 1 // function 16
                    + 2 // first register
                    + 2 // count of number of written registers
                    + 2 // crc
                    ;
  readResponse(bytesExpected);
}


void Gripper::readStatus() {
  PRINT("gripper read >> ");

  WRITE_START;
  // slave id (09)
  WRITE_LOW(GRIPPER_SLAVE_ID);
  // read (04)
  WRITE_LOW(GRIPPER_FUNCTION_READ_REGISTER);
  // starting register (07D0)
  WRITE_HIGH(GRIPPER_FIRST_INPUT_REGISTER);
  WRITE_LOW(GRIPPER_FIRST_INPUT_REGISTER);

  // registers requested (0003)
  uint16_t registersRequested=2;//GRIPPER_MAX_INPUT_REGISTERS;
  WRITE_HIGH(registersRequested);
  WRITE_LOW(registersRequested);
  // CRC (B1CE)
  WRITE_LOW_NOCRC(crc);
  WRITE_HIGH_NOCRC(crc);
  WRITE_END;
  
  int bytesExpected = 1 // slave id (should be 09)
                    + 1 // function (should be 04)
                    + 1 // bytes to follow (should be GRIPPER_MAX_OUTPUT_BYTES, aka GRIPPER_MAX_OUTPUT_REGISTERS*2)
                    + registersRequested*2  // requested number of registers * bytes per register
                    + 2 // crc
                    ;
  readResponse(bytesExpected);

  uint8_t a = inBuf[4];
  uint8_t b = inBuf[5];
/*
  // action request bytes.  high to activate.
  #define GRIPPER_BIT_rACT             0  // enable pin?  Reset to clear fault status.
  #define GRIPPER_BIT_rGTO             3  // go to desired position.
  #define GRIPPER_BIT_rATR             4  // automatic release - slow opening of all grippers to maximum.  Emergency auto-release.
  #define GRIPPER_BIT_rARD             5  // auto release direction.  high for open, low for close.
*/
#define TEST(_x,_y) ((_x >> _y) & 0x1 == 0x1)

  
  /*
  // activation status.
  #define GRIPPER_BIT_gACT             0
  // goto status.  0 for stopped (or performing activation / automatic release).
  #define GRIPPER_BIT_gGTO             3
  // 0x00 - Gripper is in reset (or automatic release) state. See Fault Status if gripper is activated.
  // 0x01 - Activation in progress.
  // 0x02 - Not used.
  // 0x03 - Activation is completed.
  #define GRIPPER_BIT_gSTA1            4 
  #define GRIPPER_BIT_gSTA2            5
  // 0x00 - Fingers are in motion towards requested position. No object detected.
  // 0x01 - Fingers have stopped due to a contact while opening before requested position. Object detected opening.
  // 0x02 - Fingers have stopped due to a contact while closing before requested position. Object detected closing.
  // 0x03 - Fingers are at requested position. No object detected or object has been loss / dropped.
  #define GRIPPER_BIT_gOBJ1            6
  #define GRIPPER_BIT_gOBJ2            7
  */
  PRINT("g");
  PRINT(TEST(a,GRIPPER_BIT_gOBJ2)?"1":"0");
  PRINT(TEST(a,GRIPPER_BIT_gOBJ1)?"1":"0");
  PRINT(TEST(a,GRIPPER_BIT_gSTA2)?"1":"0");
  PRINT(TEST(a,GRIPPER_BIT_gSTA1)?"1":"0");
  PRINT(TEST(a,GRIPPER_BIT_gGTO)?"1":"0");
  PRINT("--");
  PRINT(TEST(a,GRIPPER_BIT_gACT)?"1":"0");
  
  /*
#define GRIPPER_REG_FAULT_STATUS       2
  // 0x00 - No fault (solid blue LED)
  // # Priority faults (solid blue LED)
  // 0x05 - Action delayed; the activation (re-activation) must be completed prior to perform the action.
  // 0x07 - The activation bit must be set prior to performing the action.
  // # Minor faults (solid red LED)
  // x08 - Maximum operating temperature exceeded (≥ 85 °C internally); let cool down (below 80 °C).
  // 0x09 - No communication during at least 1 second.
  // # Major faults (LED blinking red/blue) - Reset is required (rising edge on activation bit (rACT) needed).
  // 0x0A - Under minimum operating voltage.
  // 0x0B - Automatic release in progress.
  // 0x0C - Internal fault, contact support@robotiq.com
  // 0x0D - Activation fault, verify that no interference or other error occurred.
  // 0x0E - Overcurrent triggered.
  // 0x0F - Automatic release completed.
  #define GRIPPER_BIT_gFLT0            0
  #define GRIPPER_BIT_gFLT1            1
  #define GRIPPER_BIT_gFLT2            2
  #define GRIPPER_BIT_gFLT3            3
  // per-gripper values.
  #define GRIPPER_BIT_kFLT0            4
  #define GRIPPER_BIT_kFLT1            5
  #define GRIPPER_BIT_kFLT2            6
  #define GRIPPER_BIT_kFLT3            7
  */
  PRINT("g");
  PRINTHEX((b&0xF));
  PRINT("k");
  PRINTHEX((b>>4));
}

void Gripper::readResponse(int bytesExpected) {
  arrived=0;
  //PRINT("<< ");
  long t = millis();
  
  if(gripperSerial.available()) {
    uint8_t b = gripperSerial.read();
    inBuf[arrived++] = b;
    //PRINTHEX(b);
    //PRINT(' ');
  }
  
  if(arrived==bytesExpected) {
    // expected message received
    /* The CRC field is appended to the message as the last field in the message. 
     * When this is done, the low-order byte of the field is appended first, 
     * followed by the high-order byte. The CRC high-order byte is the last 
     * byte to be sent in the message. 
     */

    uint16_t crc = crc2(inBuf,arrived-2);
    
    if(lowByte(crc)!=inBuf[arrived-2] || highByte(crc)!=inBuf[arrived-1]) {
      // message bad!
      PRINT("expected ");
      PRINTHEX(lowByte(crc));
      PRINT(' ');
      PRINTHEX(highByte(crc));
      PRINT(' ');
    } else {
      // message good
      PRINT("CRC OK ");
    }
    if(inBuf[0]!=GRIPPER_SLAVE_ID) {
      PRINT("Bad slave id");
    }
  }

  PRINT('\n');
}

uint16_t Gripper::crc1(uint8_t bytes,uint16_t len) {
  uint16_t crc=0xFFFF;
  for(int i=0;i<arrived-2;++i) {
    crc = crc16_update(crc,inBuf[i]);
  }
  return crc;
}

unsigned char auchCRCHi[] = { 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,0x40} ;
unsigned char auchCRCLo[] = { 0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,0x40};

uint16_t Gripper::crc2(uint8_t *bytes,uint16_t len) {
  unsigned char uchCRCHi = 0xFF ; // high byte of CRC initialized
  unsigned char uchCRCLo = 0xFF ; // low byte of CRC initialized
  unsigned uIndex ; /* will index into CRC lookup table  */ 
  
  while (len--) {
    uIndex = uchCRCLo ^ *bytes++ ; /* calculate the CRC  */ 
    uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex] ; 
    uchCRCHi = auchCRCLo[uIndex] ;
  }
  
  return (uchCRCHi << 8 | uchCRCLo) ; 
}


#endif // HAS_GRIPPER

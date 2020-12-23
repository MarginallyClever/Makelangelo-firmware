#pragma once
//------------------------------------------------------------------------------
// ROBOTIQ HAND-E gripper module for Sixi Robot
// 2020-03-28 dan@marginallyclever.com
// CC-by-NC-SA
//------------------------------------------------------------------------------

#ifdef HAS_GRIPPER

// expects gripper to be on a UART to RS485 module connected on pins 12 & 13.
#define GRIPPER_RX        12
#define GRIPPER_TX        13

class Gripper {
public:
  // destination buffer for responses
  uint8_t outBuf[16];
  int packed=0;
  uint8_t inBuf[16];
  int arrived=0;

  void setup();
  
  void update();
  
  void sendPositionRequest(uint8_t positionRequest,uint8_t speedRequest,uint8_t forceRequest);
  void sendActionRequest(uint8_t actionRequest);
  void readStatus();
  void readResponse(int bytesExpected);
  uint16_t crc1(uint8_t bytes,uint16_t len);
  uint16_t crc2(uint8_t *bytes,uint16_t len);
  uint16_t crc16_update(uint16_t crc, uint8_t a);
};

extern Gripper gripper;

#endif // HAS_GRIPPER

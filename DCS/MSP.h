#ifndef MSP_H_INCLUDED
#define MSP_H_INCLUDED

#include "Arduino.h"

#define MSPMSGMIN                100
#define MSP_IDENT                100   //out message version/type/etc
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_ATTITUDE             108   //out message computed att
#define MSP_MOTOR                104   //out message motor values
#define MSP_PID                  112   //out message PID values
#define MSP_MOTOR                104   //out message motor values
#define MSP_LOGDATA              198   //out message for logging, not MSP standard
#define MSP_DEBUG                199   //out message for debugging, not MSP standard
#define MSPMSGMAX                199
#define MSP_SET_PID              202   //in  message set PID values

void initMSP();
void mspWriteStart(uint8_t msgNum);
void mspWriteByte(uint8_t theByte);
void mspWriteWord(uint16_t theWord);
void mspWriteEnd();

void mspRead();
void mspWrite();

#define BUFLEN (1024)

enum MSPStates {MSPSTATE_BEGIN, MSPSTATE_DOLLAR, MSPSTATE_M, MSPSTATE_LT, MSPSTATE_DATA};
struct MSPInBuf
{
  uint8_t buf[BUFLEN];
  int bufPos;
  MSPStates state;
  int bytesRemain;
};

#endif


#define TEENSY

#include "MSP.h"

#include "RX.h"
#include "IMU.h"
#include "Motors.h"
#include "PID.h"
#include "Debug.h"

MSPInBuf mspInBuf1, mspInBuf2;

uint8_t mspOutBuf[BUFLEN];
int mspBufPosOut = 0;
bool mspMsgRequested[MSPMSGMAX - MSPMSGMIN + 1];

void initMSP() {
  for (int i=MSPMSGMIN; i<MSPMSGMAX; i++)
    mspMsgRequested[i] = false;

   mspInBuf1.state = MSPSTATE_BEGIN;
   mspInBuf1.bufPos = 0;
   mspInBuf2.state = MSPSTATE_BEGIN;
   mspInBuf2.bufPos = 0;

  Serial.begin(115200);
#ifdef TEENSY
  Serial3.begin(115200);
#endif

}

void mspWriteStart(uint8_t msgNum)
{
  mspBufPosOut = 0;
  mspOutBuf[mspBufPosOut++] = '$';
  mspOutBuf[mspBufPosOut++] = 'M';
  mspOutBuf[mspBufPosOut++] = '>';
  mspBufPosOut++; // leave room for size
  mspOutBuf[mspBufPosOut++] = msgNum;
}

void mspWriteByte(uint8_t theByte)
{
  mspOutBuf[mspBufPosOut++] = theByte;
}

void mspWriteWord(uint16_t theWord)
{
  mspOutBuf[mspBufPosOut++] = theWord & 0xff;  
  mspOutBuf[mspBufPosOut++] = theWord >> 8;  
}

void mspWriteEnd()
{
  mspOutBuf[3] = mspBufPosOut - 5; // size

  // checksum
  uint8_t crc = 0;
  for (int i=3; i<mspBufPosOut; i++)
    crc ^= (mspOutBuf[i] & 0xff);
  mspOutBuf[mspBufPosOut++] = crc;  // checksum

  for (int i=0; i<mspBufPosOut; i++)
  {
    Serial.write(mspOutBuf[i]);
    //if (mspOutBuf[4] != MSP_DEBUG)
      Serial3.write(mspOutBuf[i]);
  }
}

//-------------------------------------------------------

void writeMSP_IDENT() {
    mspWriteStart(MSP_IDENT);
    mspWriteByte(23); // version
    mspWriteByte(3);  // type - quadx
    mspWriteByte(0);  // msp version
    mspWriteWord(0);  // capabilities 0
    mspWriteWord(0);  // capabilities 1
    mspWriteEnd();
}

void evaluateMSPCmd(MSPInBuf* buf) {
  int16_t cmd = (int16_t)buf->buf[0];
  if ((MSPMSGMIN <= cmd) && (cmd <= MSPMSGMAX))
  {
    mspMsgRequested[cmd-MSPMSGMIN] = true;
  } else {
    digitalWrite(13,HIGH);
    delay(20);
    digitalWrite(13,LOW);
    switch (cmd) {
      case (MSP_SET_PID) : readMSP_SET_PID(buf); break;
    }
  }
}

void mspProcessChar(int16_t ch, MSPInBuf* buf)
{
  switch (buf->state) {
    case (MSPSTATE_BEGIN)  : buf->state = (ch == '$') ? MSPSTATE_DOLLAR : MSPSTATE_BEGIN; break;
    case (MSPSTATE_DOLLAR) : buf->state = (ch == 'M') ? MSPSTATE_M      : MSPSTATE_BEGIN; break;
    case (MSPSTATE_M)      : buf->state = (ch == '<') ? MSPSTATE_LT     : MSPSTATE_BEGIN; break;
    case (MSPSTATE_LT)     : 
      if (ch > 100) {
        buf->state = MSPSTATE_BEGIN;
      } else {
        buf->state = MSPSTATE_DATA;
        buf->bytesRemain = ch + 2; // data+cmd+chksum
        buf->bufPos = 0;
      }
      break;
    case (MSPSTATE_DATA)   :
      buf->buf[buf->bufPos++] = ch;
      if (--buf->bytesRemain == 0)
      {
        buf->state = MSPSTATE_BEGIN;
        evaluateMSPCmd(buf);
      }
      break;
  }
}

void mspRead() {
  // process usb chars
  while (Serial.available())
  {
    int16_t ch = Serial.read();
    mspProcessChar(ch, &mspInBuf1);
  }

  // process bluetooth chars
  while (Serial3.available())
  {
    int16_t ch = Serial3.read();
    mspProcessChar(ch, &mspInBuf2);
  }
}  

void mspWrite() {
  // send these messages every time
  //mspMsgRequested[MSP_IDENT-MSPMSGMIN]  = true;
  //mspMsgRequested[MSP_RAW_IMU-MSPMSGMIN]  = true;
  //mspMsgRequested[MSP_RC-MSPMSGMIN]       = true;
  //mspMsgRequested[MSP_ATTITUDE-MSPMSGMIN] = true;
  //mspMsgRequested[MSP_MOTOR-MSPMSGMIN]    = true;
  //mspMsgRequested[MSP_DEBUG-MSPMSGMIN]    = true;

  // send each requested message
  for (int i=MSPMSGMIN; i<=MSPMSGMAX; i++)
  {
    if (mspMsgRequested[i-MSPMSGMIN])
    {
      switch (i) {
        case (MSP_IDENT)    : writeMSP_IDENT();    break;   // version/type
        case (MSP_RAW_IMU)  : writeMSP_RAW_IMU();  break;   // gyro/accel data
        case (MSP_RC)       : writeMSP_RC();       break;   // rc input message
        case (MSP_ATTITUDE) : writeMSP_ATTITUDE(); break;   // yaw/roll/pitch
        case (MSP_MOTOR)    : writeMSP_MOTOR();    break;   // motor msg
        case (MSP_PID)      : writeMSP_PID();      break;   // PID values
        case (MSP_DEBUG)    : writeMSP_DEBUG();    break;   // debug values
        //case (MSP_LOGDATA)  : writeMSP_LOG();      break;   // log
      }
      mspMsgRequested[i-MSPMSGMIN]= false;
    }
  }
}


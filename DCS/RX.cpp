#include "Arduino.h"
#include "RX.h"
#include "MSP.h"

#define RC_CHANS (12)
volatile uint16_t rcValue[RC_CHANS] = {1000, MIDRC, MIDRC, MIDRC, MIDRC, MIDRC, MIDRC, MIDRC, MIDRC, MIDRC, MIDRC, MIDRC}; // throttle off

#define RX_BUFFER_SIZE (16)
static volatile uint8_t rxBufPos;
static uint8_t rxBuf[RX_BUFFER_SIZE];

extern int16_t debugVals[4];

void initRX() {
  // serial port for spektrum sat rx
  Serial1.begin(115200);  
}

// check for new frame, i.e. more than 2.5ms passed
uint32_t spekTimeLast = 0;
void checkForNewFrame() {
  uint32_t spekTimeNow = micros();
  uint32_t spekInterval = spekTimeNow - spekTimeLast;
  spekTimeLast = spekTimeNow;
  if (spekInterval > 2500) {
    rxBufPos = 0;
  }
}

// parse raw serial data into channels
void parseRXData() {
  // convert to channel data in the 1000-2000 range
    #define SPEK_FRAME_SIZE 16       // 1024 mode
  #define SPEK_CHAN_SHIFT  2       // Assumes 10 bit frames, that is 1024 mode.
  #define SPEK_CHAN_MASK   0x03    // Assumes 10 bit frames, that is 1024 mode.
  for (int b = 2; b < SPEK_FRAME_SIZE; b += 2)
  {
    uint8_t bh = rxBuf[b];
    uint8_t bl = rxBuf[b+1];
    uint8_t spekChannel = 0x0F & (bh >> SPEK_CHAN_SHIFT);
    if (spekChannel < RC_CHANS) rcValue[spekChannel] = 988 + (((uint16_t)(bh & SPEK_CHAN_MASK) << 8) + bl);
  }

  // constrain outputs and implement deadband
  #define DEADBAND (10)
  #define DEADTOP (MIDRC + DEADBAND)
  #define DEADBOT (MIDRC - DEADBAND)
  for (int i=0; i<4; i++)
  {
    rcValue[i] = constrain(rcValue[i], MINRC, MAXRC);
//    if ((rcValue[i] > DEADBOT) && (rcValue[i] < DEADTOP))
//      rcValue[i] = MIDRC;
//    if (rcValue[i] > DEADTOP)
//      rcValue[i] -= DEADBAND;
//    if (rcValue[i] < DEADBOT)
//      rcValue[i] += DEADBAND;
  } 
}

// RX interrupt
void serialEvent1() {
  checkForNewFrame();
  
  // put the data in buffer
  while ((Serial1.available()) && (rxBufPos < RX_BUFFER_SIZE))
    rxBuf[rxBufPos++] = (char)Serial1.read();

  // parse frame if done
  if (rxBufPos == SPEK_FRAME_SIZE)
    parseRXData();
}

void writeMSP_RC() {
  // write an RC msg
  mspWriteStart(MSP_RC);
  for (int i=0; i<6; i++)
  {
    mspWriteWord(rcValue[i]);
  }
  for (int i=6; i<16; i++)
  {
    mspWriteWord((uint16_t)0); // null values up to the required 16
  }
  mspWriteEnd();
}
  


#include "Arduino.h"

#include "MSP.h"

#define LOGSIZE (64)
int16_t logData[LOGSIZE][4];
int8_t logDataNdx = 0;

void writeMSP_LOG()
{
  logDataNdx = (logDataNdx + 1) % LOGSIZE;

  mspWriteStart(MSP_LOGDATA);
  for (int i=0; i<4; i++)
  {
    mspWriteWord(logData[logDataNdx][i]);
  }
  mspWriteEnd();
}


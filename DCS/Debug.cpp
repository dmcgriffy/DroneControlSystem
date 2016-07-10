#include "Arduino.h"
#include "MSP.h"

int16_t debugVals[4];

void writeMSP_DEBUG()
{
    mspWriteStart(MSP_DEBUG);
    for (int i=0; i<4; i++)
    {
      mspWriteWord(debugVals[i]);
    }
    mspWriteEnd();
}


#include "Arduino.h"

#include "Config.h"
#include "MSP.h"
#include "RX.h"

extern float axisCmd[];
extern int16_t debugVals[];

#define MYPIDITEMS 4
float axisCmdPID[MYPIDITEMS] = { 0.0f, 0.0f, 0.0f, 0.0f };
float pid_p[MYPIDITEMS] = { 1.0f, 0.8f,   0.8f,   3.0f }; // 4p & 0.5d yaw
float pid_i[MYPIDITEMS] = { 0.0f, 0.03f,  0.03f,  0.07f };
float pid_d[MYPIDITEMS] = { 0.0f, 0.3f,   0.3f,   0.9f };
uint16_t tpaCutoff = 500;
float tpaMult = 1.25f;

float prevE[MYPIDITEMS] = { 0, 0, 0, 0 };
float pid_isum[MYPIDITEMS] = { 0, 0, 0, 0 };
float pid_ddt[MYPIDITEMS] = { 0, 0, 0, 0 };
float maxDDT[MYPIDITEMS] = { 0, 0, 0, 0 };
float minDDT[MYPIDITEMS] = { 0, 0, 0, 0 };
void doPID() {
  float e;
  float tpa = (axisCmd[0] < tpaCutoff) ? ((tpaMult - 1.0f) * ((float)(tpaCutoff - axisCmd[0]) / (float)(tpaCutoff))) + 1.0f : 1.0f;
  for (int i=0; i<MYPIDITEMS; i++)
  {
    e = axisCmd[i];
    pid_isum[i] = constrain(pid_isum[i] + (e), -0.1f, 0.1f); // constrain to stop windup
    pid_ddt[i] = (e - prevE[i]) / deltat;
    if (pid_ddt[i] < minDDT[i]) minDDT[i] = pid_ddt[i];
    if (pid_ddt[i] > maxDDT[i]) maxDDT[i] = pid_ddt[i];
    //if ((i==1) || (i==2)) // tpa on roll and pitch only
    //  axisCmdPID[i] = (tpa * pid_p[i] * e) + (pid_i[i] * pid_isum[i]) + (tpa * pid_d[i] * pid_ddt[i]);
    //else
      axisCmdPID[i] = (pid_p[i] * e) + (pid_i[i] * pid_isum[i]) + (pid_d[i] * pid_ddt[i]);
    prevE[i] = e;
  }

  debugVals[0] = 1000 * tpa;
  debugVals[1] = axisCmd[0];
//  debugVals[2] = 1000 * pid_ddt[2];
//  debugVals[3] = 1000 * axisCmdPID[2];
}

#define PIDITEMS 10

void readMSP_SET_PID(MSPInBuf* buf) {
  int bufPos = 1; // skip cmd
    for (int i=1; i<MYPIDITEMS; i++) // zero is throttle, not in mw
    {
      pid_p[i] = (float)buf->buf[bufPos++] / 10.0f;
      pid_i[i] = (float)buf->buf[bufPos++] / 1000.0f;
      pid_d[i] = (float)buf->buf[bufPos++] / 10.0f;
    }  
}

void writeMSP_PID() {
    mspWriteStart(MSP_PID);
    for (int i=1; i<MYPIDITEMS; i++) // zero is throttle, not in mw
    {
      mspWriteByte(pid_p[i] * 10);
      mspWriteByte(pid_i[i] * 1000);
      mspWriteByte(pid_d[i] * 10);
    }
    for (int i=MYPIDITEMS; i<PIDITEMS; i++) // +1 for skipping throttle
    {
      mspWriteByte((uint8_t)i); // null values up to the required amount
      mspWriteByte((uint8_t)0); // null values up to the required amount
      mspWriteByte((uint8_t)0); // null values up to the required amount
    }
    mspWriteEnd();
}


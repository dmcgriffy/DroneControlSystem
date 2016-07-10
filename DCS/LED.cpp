#include "Arduino.h"
#include "LED.h"

uint8_t BIGLEDPIN[4] = {4, 11, 15, 22};   //for a quad+: rear,right,left,front

void initLEDs() {
  for (int i=0; i<4; i++)
    pinMode(BIGLEDPIN[i], OUTPUT);
}

void allLEDsOff() {
  for (int i=0; i<4; i++)
    digitalWrite(BIGLEDPIN[i], LOW);
}

void allLEDsOn() {
  for (int i=0; i<4; i++)
    digitalWrite(BIGLEDPIN[i], HIGH);
}

void blinkAround() {
  for (int i=0; i<4; i++)
    for (int j=0; j<4; j++)
    {
      for (int k=0; k<4; k++)
        digitalWrite(BIGLEDPIN[k], (j==k)?HIGH:LOW);
      delay(60);  
    }
  allLEDsOff();
}


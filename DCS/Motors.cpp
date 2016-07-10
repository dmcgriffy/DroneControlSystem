#include "Arduino.h"

#include "MSP.h"

uint16_t motor[4];
uint8_t PWM_PIN[4] = {5, 6, 23, 21};   //for a quad+: rear,right,left,front
//uint8_t PWM_PIN[4] = {21,20, 2, 3};   //for a quad+: rear,right,left,front
//uint8_t PWM_PIN[4] = {20,21,22,23};   //for a quad+: rear,right,left,front
//uint8_t PWM_PIN[4] = {9,10,5,6};   //for a quad+: rear,right,left,front

extern uint16_t debugVals[4];

void writeMotors() {      
  for (int i=0; i<4; i++)
    analogWrite(PWM_PIN[i], (motor[i]-1000)/4);
}

void initMotors() {
  for (int i=0; i<4; i++) {
    motor[i] = 1000;
    pinMode(PWM_PIN[i],OUTPUT);
    analogWriteFrequency(PWM_PIN[i], 8000);
  }
  writeMotors();
}




#define LEDPIN (13)
#define LEDPIN_TOGGLE              digitalWrite(LEDPIN, !digitalRead(LEDPIN));
#define LEDPIN_OFF                 digitalWrite(LEDPIN, LOW);
#define LEDPIN_ON                  digitalWrite(LEDPIN, HIGH);  

void initLEDs();
void allLEDsOff();
void allLEDsOn();
void blinkAround();


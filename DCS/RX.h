#define MINRC (1200)
#define MIDRC (1500)
#define MAXRC (1800)

void initRX();
uint8_t SerialPeek();
uint8_t SerialAvailable();
void readRX();
void writeMSP_RC();


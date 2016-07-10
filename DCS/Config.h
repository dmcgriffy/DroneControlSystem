#define FASTLOOPTARGET (2000)  // length of inner control loop in microseconds
#define SLOWLOOPTARGET  (100)  // length of outer control loop in milliseconds

#define deltat      ((float)FASTLOOPTARGET / 1000000.0f)         // sampling period in seconds (shown as 2 ms)
#define deltat_inv  (1.0f / deltat)                              // sampling multiplier, 1/deltat


#include "Arduino.h"
#include "i2c_t3.h"
//#include "Wire.h"

#include "Config.h"
#include "MSP.h"
#include "LED.h"

extern int16_t debugVals[4];

const int MPU=0x68;               // I2C address of the MPU-6050
unsigned int i2cErrorCnt = 0;     // running count of i2c errors

float gyroOff[3] = {0, 0, 0};   // calibration results
float accOff[3] = {0, 0, 0};    // calibration resluts
int16_t Acc[3], Gyro[3];          // raw IMU data
int16_t Tmp;                      // raw temperature data
float IMURoll,IMUPitch,IMUHead; // cooked IMU data
float   throttleCorrection;       // boost throttle for roll/pitch angle

//--- vars from Madgwick
#define gyroMeasError 3.14159265358979f * (5.0f / 180.0f)     // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError                // beta is the gain between gyro and accel
float a_x, a_y, a_z;                                          // accelerometer measurements
float w_x, w_y, w_z;                                          // gyroscope measurements in rad/s
float SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f; // estimated orientation quaternion elements with initial conditions


//---------------------
// read gyro and accel
void requestIMU()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);               // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(true);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
}

//---------------------
// read gyro only
void requestGyro()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x43);               // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(true);
  Wire.requestFrom(MPU,6,true);  // request a total of 6 registers
}

//---------------------
// read available IMU bytes and process if full message
uint8_t imuBufPos = 0;
uint8_t imuBuf[14];
void readIMU()
{
  // read the whole IMU set
  requestIMU();
  while (imuBufPos < 14) // blocks until enough data read
  {
    if (Wire.available() > 0)
    {
      imuBuf[imuBufPos++]=Wire.read();
      if (Wire.getError() != 0)
        i2cErrorCnt++;
    }
  }
  if (imuBufPos == 14)  
  {
    Acc[0]=imuBuf[0]<<8|imuBuf[1];    // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    Acc[1]=imuBuf[2]<<8|imuBuf[3];    // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    Acc[2]=imuBuf[4]<<8|imuBuf[5];    // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=imuBuf[6]<<8|imuBuf[7];       // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    Gyro[0]=imuBuf[8]<<8|imuBuf[9];   // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    Gyro[1]=imuBuf[10]<<8|imuBuf[11];  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    Gyro[2]=imuBuf[12]<<8|imuBuf[13];  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L) 
    
    //Acc[2] = -Acc[2];
    imuBufPos = 0;
  }
}

//------------------------
#define ACC_1G (2048)
void calibrateGyroAcc() {
  // --- calibrate gyro by repeatedly taking the average of blocks of readings
  //     until the XY vibration is low enough
  #define CALIBBLOCKSIZE (512)
  bool calibGood = false;
  float gyroSum[3] = {0, 0, 0};
  float accSum[3] = {0, 0, 0};
  float vibSum;
  delay(5000);
  LEDPIN_ON;
  while (!calibGood)
  {
    // clear stuff and wait a second
    for (int i=0; i<3; i++)
    {
      gyroSum[i] = 0;
      accSum[i] = 0;
    }
    vibSum = 0;
    delay(1000);

    // read a block of acc/gyro data, computing horizontal vibration
    readIMU();
    for (int i=0; i<CALIBBLOCKSIZE; i++)
    {
      delay(3);
      readIMU();
      for (int j=0; j<3; j++)
      {
        gyroSum[j] += Gyro[j];
        accSum[j] += Acc[j];
      }
      vibSum = abs(a_x + accOff[0]) + abs(a_y + accOff[1]);// + abs(a_z - accOff[2]);
    }
    for (int j=0; j<3; j++)
    {
      gyroOff[j] = -(gyroSum[j] / CALIBBLOCKSIZE);
      accOff[j]  = -(accSum[j]  / CALIBBLOCKSIZE);
    }
    calibGood = vibSum < 1.0f;
  }
  accOff[2] += ACC_1G;
  LEDPIN_OFF;
}

//------------------------------
// setup the IMU modules and do calibration
void initIMU()
{
  // --- initialize the MPU6050
  Wire.begin();
  Wire.setClock(400000);
  //TWBR = ((F_CPU / 400000) - 16) / 2;          // set the I2C clock rate to 400kHz
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);Wire.write(0x80);     // reset to default
  Wire.endTransmission(true);
  delay(50);
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);Wire.write(0x03);     // SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
  Wire.endTransmission(true);
  delay(50);
  Wire.beginTransmission(MPU);
  Wire.write(0x1a);Wire.write(0x03);     // EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
  Wire.write(0x1b);Wire.write(0x18);     // FS_SEL = 3: Full scale set to 2000 deg/sec
  Wire.write(0x1c);Wire.write(0x10);     // ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
  Wire.endTransmission(true);
  requestIMU();

  calibrateGyroAcc();
}


//---------------------------------
// calcIMU -- the main update call
#define gyro2radpersec  (2000.0f / 32768.0f) * (3.1415926f / 180.0f)  // scale raw gyro @2000 deg/sec full scale to rad/sec
float rad2deg = (180.0f / 3.1415926f);
void calcIMU()
{
    a_x = (float)Acc[0] + accOff[0];
    a_y = (float)Acc[1] + accOff[1];
    a_z = (float)Acc[2] + accOff[2];
    w_x = ((float)Gyro[0] + gyroOff[0]) * gyro2radpersec;
    w_y = ((float)Gyro[1] + gyroOff[1]) * gyro2radpersec;
    w_z = ((float)Gyro[2] + gyroOff[2]) * gyro2radpersec;
    
    //--- Madgwick code ---
    // Local system variables
    float norm;                                                           // vector norm denominator
    float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derrivative from gyroscopes elements
    float f_1, f_2, f_3;                                                  // objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33;             // objective function Jacobian elements
    float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4;             // estimated direction of the gyroscope error
    // Axulirary variables to avoid reapeated calcualtions
    float halfSEq_1 = 0.5f * SEq_1;
    float halfSEq_2 = 0.5f * SEq_2;
    float halfSEq_3 = 0.5f * SEq_3;
    float halfSEq_4 = 0.5f * SEq_4;
    float twoSEq_1 = 2.0f * SEq_1;
    float twoSEq_2 = 2.0f * SEq_2;
    float twoSEq_3 = 2.0f * SEq_3;
    // Normalise the accelerometer measurement
    norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    a_x /= norm;
    a_y /= norm;
    a_z /= norm;
    throttleCorrection = a_z;

    // Compute the quaternion derrivative measured by gyroscopes
    SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
    SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
    SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
    SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
    // only apply accels if .75 < g < 1.25
    if ((0.75f < a_z) && (a_z < 1.25f))
    {
      // Compute the objective function and Jacobian
      f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
      f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
      f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
      J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
      J_12or23 = 2.0f * SEq_4;
      J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
      J_14or21 = twoSEq_2;
      J_32 = 2.0f * J_14or21; // negated in matrix multiplication
      J_33 = 2.0f * J_11or24; // negated in matrix multiplication
      // Compute the gradient (matrix multiplication)
      SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
      SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
      SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
      SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;
      // Normalise the gradient
      norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
      SEqHatDot_1 /= norm;
      SEqHatDot_2 /= norm;
      SEqHatDot_3 /= norm;
      SEqHatDot_4 /= norm;
      // Compute then integrate the estimated quaternion derrivative
      SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
      SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
      SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
      SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
    }
    else
    {
      // Compute then integrate the estimated quaternion derrivative
      // with no accel
      SEq_1 += (SEqDot_omega_1) * deltat;
      SEq_2 += (SEqDot_omega_2) * deltat;
      SEq_3 += (SEqDot_omega_3) * deltat;
      SEq_4 += (SEqDot_omega_4) * deltat;
    }
    // Normalise quaternion
    norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
    SEq_1 /= norm;
    SEq_2 /= norm;
    SEq_3 /= norm;
    SEq_4 /= norm;
    //--- end Madgwick code ---

  //debugVals[0] = (int)(SEq_1 * 1000.0f);
  //debugVals[1] = (int)(SEq_2 * 1000.0f);
  //debugVals[2] = (int)(SEq_3 * 1000.0f);
  //debugVals[3] = (int)(SEq_4 * 1000.0f);

    // euler angles
//  IMUPitch = rad2deg * atan2(2.0f * ((SEq_1*SEq_2) + (SEq_3*SEq_4)), 1.0f - (2.0f * ((SEq_2*SEq_2) + (SEq_3*SEq_3))));
//  IMUHead  = -rad2deg * atan2(2.0f * ((SEq_1*SEq_4) + (SEq_2*SEq_3)), 1.0f - (2.0f * ((SEq_3*SEq_3) + (SEq_4*SEq_4))));
//  IMURoll  = rad2deg * asin(2.0f * ((SEq_1*SEq_3) + (SEq_4*SEq_2)));
    // tait-b,,, angles
    IMUPitch = -rad2deg * atan2(2.0*(SEq_3*SEq_4 + SEq_1*SEq_2), SEq_1*SEq_1 - SEq_2*SEq_2 - SEq_3*SEq_3 + SEq_4*SEq_4);
    IMURoll = rad2deg * asin(-2.0*(SEq_2*SEq_4 - SEq_1*SEq_3));
    IMUHead = -rad2deg * atan2(2.0*(SEq_2*SEq_3 + SEq_1*SEq_4), SEq_1*SEq_1 + SEq_2*SEq_2 - SEq_3*SEq_3 - SEq_4*SEq_4);
//    IMUPitch = 0.0f;
//    IMUHead = 0.0f;
//    IMURoll = 0.0f;
}

//---------------------
void writeMSP_RAW_IMU() {
    mspWriteStart(MSP_RAW_IMU);
    for (int i=0; i<3; i++)
    {
      mspWriteWord(Acc[i]);
    }
    mspWriteWord(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
    for (int i=0; i<3; i++)
    {
      mspWriteWord(Gyro[i]);
    }
    mspWriteEnd();  
}

//---------------------
void writeMSP_ATTITUDE() {
    mspWriteStart(MSP_ATTITUDE);
    mspWriteWord((int16_t)(IMURoll*10));
    mspWriteWord((int16_t)(IMUPitch*10));
    mspWriteWord((int16_t)(IMUHead));
    mspWriteEnd();
}


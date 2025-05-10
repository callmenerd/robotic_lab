#include "SerialTransfer.h"
#include "FastIMU.h"
#include "Madgwick.h"
#include "SerialTransfer.h"
#include <Wire.h>

#define IMU_ADDRESS 0x69
#define PERFORM_CALIBRATION
BMI160 IMU;
SerialTransfer myTransfer;

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;
MagData magData;
Madgwick filter;

const float driftCompensate = 0.98;
const int sampling = 16;
int countUP, countDN = 0;
float gx, gy, gz;
float prevgx, prevgy, prevgz = 0;
float gmean[3] = {0,0,0};

float q[4] = {0,0,0,0};
float roll, pitch, yaw = 0;

struct __attribute__((packed)) STRUCTRX {
  float Vx;
  float Vy;
  float W;
} rxStruct;

struct __attribute__((packed)) STRUCTTX {
  float Vrx;
  float Vry;
  float Wr;
} txStruct;

char arr[] = "stm";
char rec[4];

void setup()
{
  Wire.begin();
  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.setClock(400000); //400khz clock
  Serial.begin(115200);
  //myTransfer.begin(Serial);
  IMU.init(calib, IMU_ADDRESS);
  #ifdef PERFORM_CALIBRATION
  
    delay(3000);
    IMU.calibrateAccelGyro(&calib);
    delay(3000);
    IMU.init(calib, IMU_ADDRESS);
    filter.begin(0.2f);
  #endif

  //err = IMU.setGyroRange(500);      //USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
  //err = IMU.setAccelRange(2);       //THESE TWO SET THE GYRO RANGE TO ±500 DPS AND THE ACCELEROMETER RANGE TO ±2g
}


void loop()
{
  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);
  if (IMU.hasTemperature()) {
	  Serial.print(IMU.getTemp());
  }
  gx = drift_elim(gyroData.gyroX, prevgx, 0);
  gy = drift_elim(gyroData.gyroY, prevgy, 1);
  gz = drift_elim(gyroData.gyroZ, prevgz, 2);
  prevgx = gyroData.gyroX;
  prevgy = gyroData.gyroY;
  prevgz = gyroData.gyroZ;
  filter.updateIMU(gx, gy, gz, accelData.accelX, accelData.accelY, accelData.accelZ);
  q[0] = filter.getQuatX(); q[1] = filter.getQuatY(); q[2] = filter.getQuatZ(); q[3] = filter.getQuatW();

  // convert aerospace quaternion to aerospace Euler
  yaw   = 180/PI * atan2(q[0]*q[1] + q[3]*q[2], 0.5 - q[1]*q[1] - q[2]*q[2]);
  pitch = 180/PI * asin(-2.0 * (q[0]*q[2] - q[3]*q[1]));
  roll  = 180/PI * atan2(q[3]*q[0] + q[1]*q[2], 0.5 - q[0]*q[0] - q[1]*q[1]);
  // txStruct.Vrx = roll;
  // txStruct.Vry = pitch;
  // txStruct.Wr = yaw;
  // if(myTransfer.available()){
  //   uint16_t sendSize = 0;
  //   sendSize = myTransfer.txObj(txStruct, sendSize);
  //   myTransfer.sendData(sendSize);
  //   uint16_t recSize = 0;
  //   recSize = myTransfer.rxObj(rxStruct, recSize);
  // }
}

float drift_elim(float raw, float prevraw, int idx){
  float mean = 0;
  if(abs(raw) <= 0.5){
    if(abs(raw)*driftCompensate >= abs(prevraw)){
      if(countUP >= sampling){
        countUP = 0;
        mean = gmean[idx]/sampling;
        gmean[idx] = 0;
      } else{
        countDN = 0;
        countUP++;
        gmean[idx] += raw;
      }
    } else if(abs(raw)*driftCompensate <= abs(prevraw)){
      if(countDN >= sampling){
        countDN = 0;
        mean = gmean[idx]/sampling;
        gmean[idx] = 0;
      } else{
        countUP = 0;
        countDN++;
        gmean[idx] += raw;
      }
    } else{
      countUP = 0;
      countDN = 0;
      gmean[idx] = 0;
    }
  } else{
    countUP = 0;
    countDN = 0;
    mean = raw;
  }
  return mean;
}

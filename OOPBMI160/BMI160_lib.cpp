#include "BMI160_lib.h"

BMI160_lib::BMI160_lib() {
  return;
}

void BMI160_lib::set_sampling(int iter){
  this->sampling = iter;
}

void BMI160_lib::set_drift_compensate(float drift_constant){
  this->driftCompensate = drift_constant;
}

float BMI160_lib::drift_elim(float raw, float prevraw, int idx){
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

void BMI160_lib::setup() {
  Wire.begin();
  Wire.setClock(400000); //400khz clock
  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    while (true) {
      ;
    }
  }
  
#ifdef PERFORM_CALIBRATION
  if (IMU.hasMagnetometer()) {
    IMU.calibrateMag(&calib);
  }
  else {
    delay(1000);
  }
  IMU.calibrateAccelGyro(&calib);
  delay(1000);
  IMU.init(calib, IMU_ADDRESS);
  filter.begin(0.2f);
#endif

  //err = IMU.setGyroRange(500);      //USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
  //err = IMU.setAccelRange(2);       //THESE TWO SET THE GYRO RANGE TO ±500 DPS AND THE ACCELEROMETER RANGE TO ±2g
  
  if (err != 0) {
    while (true) {
      ;
    }
  }
}

void BMI160_lib::updateValue() {
  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);
  // if (IMU.hasTemperature()) {
	//   Serial.print(IMU.getTemp());
  // }
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
}
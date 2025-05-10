#ifndef BMI160_LIB_H
#define BMI160_LIB_H
#include "FastIMU.h"
#include "Madgwick.h"
#include <Wire.h>

#define IMU_ADDRESS 0x69
#define PERFORM_CALIBRATION

class BMI160_lib{
  private:
    BMI160 IMU;

    calData calib = { 0 };  //Calibration data
    AccelData accelData;    //Sensor data
    GyroData gyroData;
    MagData magData;
    Madgwick filter;

    int countUP, countDN = 0;
    float gx, gy, gz;
    float prevgx, prevgy, prevgz = 0;
    float gmean[3] = {0,0,0};

    float q[4] = {0,0,0,0};
    float drift_elim(float raw, float prevraw, int idx);
  public:
    float roll, pitch, yaw = 0;
    float driftCompensate = 0.98;
    int sampling = 16;
    BMI160_lib(void);
    void setup();
    void set_sampling(int iter);
    void set_drift_compensate(float drift_constant);
    void updateValue();
};

#endif // !BMI160_H
/*
 This sketch shows to rotate the heading (yaw angle) of the estimated orientation.
 */

#include <basicMPU6050.h>       // Library for IMU sensor. See this link: https://github.com/RCmags/basicMPU6050
#include <imuFilter.h>

basicMPU6050<> imu;

imuFilter fusion;

// Variables for drift correction
const float driftCompensate = 1;
const int sampling = 8;
int countUP, countDN = 0;
float gx, gy, gz;
float prevgx, prevgy, prevgz = 0;
float gmean[3] = {0,0,0};

void setup() {
  Serial.begin(115200);
  // Wire.setSDA(0);
  // Wire.setSCL(1);
  Wire.requestFrom(0x69, 1);
  Wire.begin();
  // Calibrate imu
  imu.setup();
  imu.setBias();
  
  // Initialize filter: 
  fusion.setup( imu.ax(), imu.ay(), imu.az() );     
                  
  // Rotate heading:
  // float angle = 45 * DEG_TO_RAD;                // angle in radians to rotate heading about z-axis
  // fusion.rotateHeading( angle, SMALL_ANGLE );   // Can choose LARGE_ANGLE or SMALL_ANGLE approximation
  Serial.println("START");
}

void loop() {  
  // Update filter:
  fusion.update( imu.gx(), imu.gy(), imu.gz(), imu.ax(), imu.ay(), imu.az() );    
  gx = drift_elim(imu.gx(), prevgx, 0);
  gy = drift_elim(imu.gy(), prevgy, 1);
  gz = drift_elim(imu.gz(), prevgz, 2);
  prevgx = imu.gx();
  prevgy = imu.gy();
  prevgz = imu.gz();

  // Serial.print(imu.gx());
  // Serial.print(" ");
  // Serial.print(imu.gy());
  // Serial.print(" ");
  // Serial.print(imu.gz());
  // Serial.println(" ");

  // Serial.print(gx*100000);
  // Serial.print(" ");
  // Serial.print(gy*100000);
  // Serial.print(" ");
  // Serial.print(gz*100000);
  // Serial.println(" ");

  //Display angles:
  Serial.print(fusion.pitch()*180/PI);
  Serial.print(" ");
  Serial.print(fusion.yaw()*180/PI);
  Serial.print(" ");
  Serial.print(fusion.roll()*180/PI);
  Serial.println();
}

float drift_elim(float raw, float prevraw, int idx){
  float mean = 0;
  if(abs(raw) <= 0.0005){
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

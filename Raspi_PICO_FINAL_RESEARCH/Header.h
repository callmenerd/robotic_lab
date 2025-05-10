#include <SimpleKalmanFilter.h>
#include <Mahony_BMX160.h>
#include <Madgwick_BMX160.h>
#include <math.h>
#include "CalibrateIMU.h"
#include "SerialTransfer.h"

// ---- OBJECT DECLARATION --- //
CalibrateIMU BMX;
//Mahony_BMX160 filter;
Madgwick_BMX160 filter;
// SerialTransfer
SerialTransfer myTransfer;

// ---- INPUT ---- //
const int input_rate = 5;
unsigned long input_prev = 0;
const int sensor_rate = 5;
unsigned long sensor_prev = 0;
unsigned long prevcekwaktu,cekwaktu = 0;

// ---- IMU ---- //
const float driftCompensate = 0.995;
const float drift_thresh = 0.5;
const float g_earth = 9.8067;
const int sampling = 16;
int countUP, countDN = 0;
float gx, gy, gz;
float ax, ay, az;
float prevgx, prevgy, prevgz = 0;
float init_x, init_y, init_z = 0;
float savedgx, savedgy, savedgz = 0.0;
float roll, pitch, heading = 0;
float gmean[3] = {0,0,0};

// ---- COMMAND SIGNAL ---- //
char machine_state = ' ';
char mechanism_state = ' ';
char prev_machine_state = ' ';
char prev_mechanism_state = ' ';
char prevMecState = ' ';

// ---- UART Message ---- //
struct __attribute__((packed)) STRUCTRX {
  char mechanism_state;
  char command;
} rxStruct;

struct __attribute__((packed)) STRUCTTX {
  float timestamp;
  float roll; float pitch; float yaw;
  float gx; float gy; float gz;
  float ax; float ay; float az;
  char feedback_respon;
} txStruct;

/* ---- KALMAN FILTER ---- */
/*e_mea: Measurement Uncertainty 
  e_est: Estimation Uncertainty 
  q: Process Noise*/
float e_mea = 0.02, q = 0.005, e_est = 0.02;

SimpleKalmanFilter ax_calc(e_mea, e_est, q);
SimpleKalmanFilter ay_calc(e_mea, e_est, q);
float acc[2] = {0, 0};

// ---- ZERO DRIFT COMPENSATOR --- //
float drift_elim(float raw, float prevraw, int idx){
  float mean = 0;
  if(abs(raw) <= drift_thresh){
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
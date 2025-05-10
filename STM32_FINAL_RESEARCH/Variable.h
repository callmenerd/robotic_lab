#include "Header.h"

/* ---- KINEMATICS CONSTANTS ---- */
const float a1_ideal = 45; //Sudut tiap roda relatif terhadap sumbu x positif
const float a2_ideal = 135;
const float a3_ideal = 225;
const float a4_ideal = 315;
const float a1 = 49.7334; //Sudut tiap roda relatif terhadap sumbu x positif
const float a2 = 133.8309;
const float a3 = 223.3317;
const float a4 = 313.6028;

const float r = 5; //Jari-jari roda
const float R = 23.5849; //Jari-jari roda terhadap pusat robot

/* ---- ENCODER ---- */

float Vx, Vy, W = 0;
float Vreal1, Vreal2, Vreal3, Vreal4 = 0;
float Vfilt1, Vfilt2, Vfilt3, Vfilt4 = 0;

volatile long fr_tics, fl_tics, bl_tics, br_tics;
volatile long prev_fr_tics, prev_fl_tics, prev_bl_tics, prev_br_tics;
volatile long prev_Timing, Timing = 0;

/* ---- SETPOINT ---- */
const int max_linear_speed = 188; //vel/(2*r*PI) * 60; vel in cm/s
const int max_angular_speed = 188; //vel/360*(2*R)/2*r*60; vel in deg/s
const int min_motor_pwm = 25; //pwm min for the motor to start rotating

/* ---- PID ---- */
// 1m/s -> 0.475 11/8/5 0.01/0.0125/0.015
float Output1 = 0, Kp1 = 0.75, Ki1 = 15, Kd1 = 0.01;
float Output2 = 0, Kp2 = 0.75, Ki2 = 15, Kd2 = 0.01;
float Output3 = 0, Kp3 = 0.75, Ki3 = 15, Kd3 = 0.01;
float Output4 = 0, Kp4 = 0.75, Ki4 = 15, Kd4 = 0.01;

QuickPID motor1(&Vfilt1, &Output1, &calc.Vwheel[0], Kp1, Ki1, Kd1,
               motor1.pMode::pOnError,
               motor1.dMode::dOnMeas,            
               motor1.iAwMode::iAwCondition,  
               motor1.Action::direct);
QuickPID motor2(&Vfilt2, &Output2, &calc.Vwheel[1], Kp2, Ki2, Kd2,
               motor2.pMode::pOnError,
               motor2.dMode::dOnMeas,            
               motor2.iAwMode::iAwCondition,  
               motor2.Action::direct);
QuickPID motor3(&Vfilt3, &Output3, &calc.Vwheel[2], Kp3, Ki3, Kd3,
               motor3.pMode::pOnError,
               motor3.dMode::dOnMeas,            
               motor3.iAwMode::iAwCondition,  
               motor3.Action::direct);
QuickPID motor4(&Vfilt4, &Output4, &calc.Vwheel[3], Kp4, Ki4, Kd4,
               motor4.pMode::pOnError,
               motor4.dMode::dOnMeas,            
               motor4.iAwMode::iAwCondition,  
               motor4.Action::direct);

/* ---- KALMAN FILTER ---- */
/*e_mea: Measurement Uncertainty 
  e_est: Estimation Uncertainty 
  q: Process Noise*/
float e_mea = 22, q = 0.25, e_est = 22;

SimpleKalmanFilter Roda_1(e_mea, e_est, q);
SimpleKalmanFilter Roda_2(e_mea, e_est, q);
SimpleKalmanFilter Roda_3(e_mea, e_est, q);
SimpleKalmanFilter Roda_4(e_mea, e_est, q);

/* ---- SERIAL COM ---- */
struct __attribute__((packed)) STRUCTRX {
  float Vx;
  float Vy;
  float W;
  float psi;
} rxStruct;

struct __attribute__((packed)) STRUCTTX {
  float timestamp;
  float X; float Y; float tetha;
  float Vx; float Vy; float Wr;
  char cmd;
  char team;
} txStruct;

bool prevStateStart = false;
bool prevStateRetry = false;
/* ---- SETPOINT ---- */
float target_linear_speed = 250;
float target_angular_speed = 200;

/* ---- PID ---- */
float Output1 = 0, Kp1 = 0.475, Ki1 = 8, Kd1 = 0.0;
float Output2 = 0, Kp2 = 0.475, Ki2 = 8, Kd2 = 0.0;
float Output3 = 0, Kp3 = 0.475, Ki3 = 8, Kd3 = 0.0;
float Output4 = 0, Kp4 = 0.475, Ki4 = 8, Kd4 = 0.0;

/* ---- KALMAN FILTER ---- */
/*e_mea: Measurement Uncertainty 
  e_est: Estimation Uncertainty 
  q: Process Noise*/
float e_mea = 2, q = 0.25, e_est = 0.4;

/* ---- KINEMATICS ---- */
// CONSTANTS
const float a1 = 48.5323; //Sudut tiap roda relatif terhadap sumbu x positif
const float a2 = 134.1931;
const float a3 = 221.6981;
const float a4 = 315.7252;
const float r = 5;
const float R = 23.584953;
const float PPR = 537.6;
const float K = PI*2*r;
//ENCODER TENGAH
const float a5 = -11.92;
const float a6 = 193.13;
const float PPR2 = 2400;
const float d = 5.8;
const float K2 = PI * d;

//VELOCITY VARIABLE
bool stop = false;
int Vx, Vy, W = 0;

/* ---- ENCODER ---- */
//REAL VELOCITY
float Vr1, Vr2, Vr3, Vr4 = 0;
float Vfilt1, Vfilt2, Vfilt3, Vfilt4 = 0;

volatile long prev_fr = 0;
volatile long prev_fl = 0;
volatile long prev_bl = 0;
volatile long prev_br = 0;
bool prevStateStart = false;
bool prevStateRetry = false;

volatile long prevT = 0;
volatile long posPrev = 0;

//jarak
float distanceX = 0;
float distanceY = 0;
float distanceW = 0;
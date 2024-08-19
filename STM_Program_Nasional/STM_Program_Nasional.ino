#include "Header.h"
#include "Variable.h"
#include "SerialTransfer.h"

SerialTransfer myTransfer;

struct __attribute__((packed)) STRUCTRX {
  float Vx;
  float Vy;
  float W;
  float psi;
} rxStruct;

struct __attribute__((packed)) STRUCTTX {
  float X;
  float Y;
  float Wr;
  char cmd;
  char team;
} txStruct;

/* OBJECT BUILD */
// MOVEMENT
LowerPart rangkabawah;
Kinematics calc;
QuickPID motor1(&Vfilt1, &Output1, &calc.Vw[0], Kp1, Ki1, Kd1,
               motor1.pMode::pOnError,
               motor1.dMode::dOnMeas,            
               motor1.iAwMode::iAwCondition,  
               motor1.Action::direct);
QuickPID motor2(&Vfilt2, &Output2, &calc.Vw[1], Kp2, Ki2, Kd2,
               motor2.pMode::pOnError,
               motor2.dMode::dOnMeas,            
               motor2.iAwMode::iAwCondition,  
               motor2.Action::direct);
QuickPID motor3(&Vfilt3, &Output3, &calc.Vw[2], Kp3, Ki3, Kd3,
               motor3.pMode::pOnError,
               motor3.dMode::dOnMeas,            
               motor3.iAwMode::iAwCondition,  
               motor3.Action::direct);
QuickPID motor4(&Vfilt4, &Output4, &calc.Vw[3], Kp4, Ki4, Kd4,
               motor4.pMode::pOnError,
               motor4.dMode::dOnMeas,            
               motor4.iAwMode::iAwCondition,  
               motor4.Action::direct);
/* ---- KALMAN FILTER ---- */
SimpleKalmanFilter Roda_1(e_mea, e_est, q);
SimpleKalmanFilter Roda_2(e_mea, e_est, q);
SimpleKalmanFilter Roda_3(e_mea, e_est, q);
SimpleKalmanFilter Roda_4(e_mea, e_est, q);

void setup() {
  Serial.begin(115200);
  pinMode(START_PUSH_BUTTON, INPUT_PULLUP);
  pinMode(RETRY_PUSH_BUTTON, INPUT_PULLUP);
  pinMode(TEAM_PUSH_BUTTON, INPUT_PULLUP);
  // pinMode(LimitR, INPUT_PULLUP);
  // pinMode(LimitL, INPUT_PULLUP);
  PID_init();

  //OBJECT CONSTRUCTING
  rangkabawah = LowerPart(sel_fr, pwm_fr, sel_fl, pwm_fl, sel_bl, pwm_bl, sel_br, pwm_br);
  calc = Kinematics(a1, a2, a3, a4, a5, a6, r, R);
  calc.PPR = 537.6;
  myTransfer.begin(Serial);
}

void loop() {
  if (millis() - input_prevmillis >= inputrate) {
    prevStateStart = start;
    prevStateRetry = retry;
    start = digitalRead(START_PUSH_BUTTON);
    retry = digitalRead(RETRY_PUSH_BUTTON);
    team = digitalRead(TEAM_PUSH_BUTTON);
    MoveRobot();
    input_prevmillis = millis();
  }
  PID_compute();
  Vr1 = ENCFR.velocity();
  Vr2 = ENCFL.velocity();
  Vr3 = ENCBL.velocity();
  Vr4 = ENCBR.velocity();
  // Serial.print(ENCFR.read());
  // Serial.print(" | ");
  // Serial.print(ENCFL.read());
  // Serial.print(" | ");
  // Serial.print(ENCBL.read());
  // Serial.print(" | ");
  // Serial.print(ENCBR.read());
  // Serial.print(" | ");
  // Serial.print(ENCMR.read());
  // Serial.print(" | ");
  // Serial.print(ENCML.read());
  // Serial.print(" | ");
  // Serial.print(calc.S_od[0]);
  // Serial.print(" | ");
  // Serial.print(calc.S_od[1]);
  // Serial.print(" | ");
  // Serial.print(calc.S_od[2]);
  // Serial.println("");
  Vfilt1 = Roda_1.updateEstimate(Vr1);
  Vfilt2 = Roda_2.updateEstimate(Vr2);
  Vfilt3 = Roda_3.updateEstimate(Vr3);
  Vfilt4 = Roda_4.updateEstimate(Vr4);
  txStruct.X = (float) calc.s[0];
  txStruct.Y = (float) calc.s[1];
  txStruct.Wr = (float) calc.s[2];
  txStruct.team = (char) team;
  if(!start){
    if(prevStateStart == true){
      ENCFR.readAndReset();
      ENCFL.readAndReset();
      ENCBL.readAndReset();
      ENCBR.readAndReset();
    }
    txStruct.cmd = 'M';}
  else if(!retry){
    if(prevStateRetry == true){
      ENCFR.readAndReset();
      ENCFL.readAndReset();
      ENCBL.readAndReset();
      ENCBR.readAndReset();
    }
    txStruct.cmd = 'R';}
  else{
    txStruct.cmd = 'S';
    rxStruct.Vx = 0;
    rxStruct.Vy = 0;
    rxStruct.W = 0;
    ENCFR.readAndReset();
    ENCFL.readAndReset();
    ENCBL.readAndReset();
    ENCBR.readAndReset();
  }
  if(myTransfer.available()){
    uint16_t sendSize = 0;
    sendSize = myTransfer.txObj(txStruct, sendSize);
    myTransfer.sendData(sendSize);

    uint16_t recSize = 0;
    recSize = myTransfer.rxObj(rxStruct, recSize);
  }
}
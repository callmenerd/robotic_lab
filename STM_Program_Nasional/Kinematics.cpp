/*----------------------------------------------
  ---------PROPERTY OF BLAKASUTHA_UNSOED--------
  ----------------------------------------------
  ==============AUTHOR:RAFIF SUSENA=============
  _____More Info : rafifsusena1@gmail.com_______
*/
#include"Kinematics.h"

//Konstruktor
Kinematics::Kinematics(){
  return;
}

Kinematics::Kinematics(float a1, float a2, float a3, float a4, float a5, float a6, float r, float R){
  Serial.begin(9600);
  this->a1 = a1; this->a2 = a2; this->a3 = a3; this->a4 = a4;
  this->a5 = a5; this->a6 = a6;
  this->r;
  this->R;
  this->Vr;
  this->Vw;
  this->s;
  this->PPR;
  this->PPR2;
}

//Inverse Kinematic -> TO GET THE VELOCITY VALUE FOR EACH MOTOR
void Kinematics::inverse_kin(float Vx, float Vy, float Wr, float xi){
  float Vw1, Vw2, Vw3, Vw4 = 0;
  Vw1 = round((-SpeedTrig.sin(xi+a1)*Vx + SpeedTrig.cos(xi+a1)*Vy + Wr));
  Vw2 = round((-SpeedTrig.sin(xi+a2)*Vx + SpeedTrig.cos(xi+a2)*Vy + Wr));
  Vw3 = round((-SpeedTrig.sin(xi+a3)*Vx + SpeedTrig.cos(xi+a3)*Vy + Wr));
  Vw4 = round((-SpeedTrig.sin(xi+a4)*Vx + SpeedTrig.cos(xi+a4)*Vy + Wr));
  //Results is in m/s (linear velocity in X, Y and linear rotation velocity)
  Vw[0] = Vw1; Vw[1] = Vw2; Vw[2] = Vw3; Vw[3] = Vw4;
}

// Get the coordinate position of the robot in X, Y and psi (heading), and robot's velocity relative to global coordinate
void Kinematics::forward_kin(float v1, float v2, float v3, float v4, float xi, bool odom){
  float x_, y_, w_ = 0;
  if(odom){
    x_ = ((v1*(-SpeedTrig.sin(xi+a1))) + (v2*(-SpeedTrig.sin(xi+a2))) + (v3*(-SpeedTrig.sin(xi+a3))) + (v4*(-SpeedTrig.sin(xi+a4))))/2;
    y_ = ((v1*(SpeedTrig.cos(xi+a1))) + (v2*(SpeedTrig.cos(xi+a2))) + (v3*(SpeedTrig.cos(xi+a3))) + (v4*(SpeedTrig.cos(xi+a4))))/2;
    w_ = (v1+v2+v3+v4)/4;
    this->s[0] = x_*10*PI/PPR; this->s[1] = y_*10*PI/PPR; this->s[2] = w_*10*PI/PPR;
  } else{
    x_ = ((v1*(-SpeedTrig.sin(xi+a1))) + (v2*(-SpeedTrig.sin(xi+a2))) + (v3*(-SpeedTrig.sin(xi+a3))) + (v4*(-SpeedTrig.sin(xi+a4))))/2;
    y_ = ((v1*(SpeedTrig.cos(xi+a1))) + (v2*(SpeedTrig.cos(xi+a2))) + (v3*(SpeedTrig.cos(xi+a3))) + (v4*(SpeedTrig.cos(xi+a4))))/2;
    w_ = (v1+v2+v3+v4)/4;
    Vr[0] = x_; Vr[1] = y_; Vr[2] = w_;
  }
}

void Kinematics::forward_pulse(float p1, float p2, float xi){
  float px, py, pw = 0;
  // Hitung perubahan sudut rotasi robot
  px = (-p1*SpeedTrig.cos(xi+119.8989) - p2*SpeedTrig.cos(xi+222.5805))/2;
  py = (p1*SpeedTrig.sin(xi+119.8989) + p2*SpeedTrig.sin(xi+222.5805))/2;
  pw = (p1*SpeedTrig.sin(xi+119.8989) - p2*SpeedTrig.sin(xi+222.5805))/18;
  // pw = (p1 * SpeedTrig.sin(135 + a5) + p2 * SpeedTrig.cos(90 + a6 - 225)) / 2.0;

  // Hitung perubahan posisi pada sumbu x dan y
  // float Vx1 = -p1 * SpeedTrig.cos(135);
  // float Vy1 = p1 * SpeedTrig.sin(135);
  // float w1 = p1 * SpeedTrig.sin(135);

  // float Vx2 = -p2 * SpeedTrig.cos(225);
  // float Vy2 = -p2 * SpeedTrig.sin(225);
  // float w2 = -p2 * SpeedTrig.sin(193.13 - 225);

  // // Menghitung VX
  // px = (-Vx1 * SpeedTrig.cos(xi) - Vy1 * SpeedTrig.sin(xi) - Vx2 * SpeedTrig.cos(xi) - Vy2 * SpeedTrig.sin(xi)) / 4;
    
  // // Menghitung VY
  // py = (-Vx1 * SpeedTrig.sin(xi) + Vy1 * SpeedTrig.cos(xi) - Vx2 * SpeedTrig.sin(xi) - Vy2 * SpeedTrig.cos(xi)) / 4;
  // pw = ((p1/this->PPR2*5.8) + (p2/this->PPR2*5.8))/9*360;
  this->S_od[0] = px; this->S_od[1] = py; this->S_od[2] = pw;
}
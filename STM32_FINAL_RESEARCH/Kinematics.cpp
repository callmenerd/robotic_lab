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

Kinematics::Kinematics(float a1, float a2, float a3, float a4, float r, float R){
  this->a1 = a1; this->a2 = a2; this->a3 = a3; this->a4 = a4;
  this->r = r;
  this->R = R;
}

void Kinematics::init(void){
  //Term of use = To cut the perform time of calculating trigonometric value for each time, the value is calculate once in the beginning
  this->sin_value[0] = isin(this->a1); this->sin_value[1] = isin(this->a2);
  this->sin_value[2] = isin(this->a3); this->sin_value[3] = isin(this->a4);

  this->cos_value[0] = icos(this->a1); this->cos_value[1] = icos(this->a2); 
  this->cos_value[2] = icos(this->a3); this->cos_value[3] = icos(this->a4);

  this->err_cos_value[0] = icos(this->ideal_a1-this->a1); this->err_cos_value[1] = icos(this->ideal_a2-this->a2);
  this->err_cos_value[2] = icos(this->ideal_a3-this->a3); this->err_cos_value[3] = icos(this->ideal_a4-this->a4);
}

void Kinematics::update_angle(float psi){
  //Term of use = To cut the perform time of calculating trigonometric value for each time, the value is calculate once in the beginning
  this->sin_value[0] = isin(this->a1+psi); this->sin_value[1] = isin(this->a2+psi);
  this->sin_value[2] = isin(this->a3+psi); this->sin_value[3] = isin(this->a4+psi);

  this->cos_value[0] = icos(this->a1+psi); this->cos_value[1] = icos(this->a2+psi); 
  this->cos_value[2] = icos(this->a3+psi); this->cos_value[3] = icos(this->a4+psi);
}

void Kinematics::set_ideal_value(float i_a1, float i_a2, float i_a3, float i_a4){
  this->ideal_a1 = i_a1;
  this->ideal_a2 = i_a2; 
  this->ideal_a3 = i_a3; 
  this->ideal_a4 = i_a4;
}

//Inverse Kinematic -> TO GET THE VELOCITY VALUE FOR EACH MOTOR
void Kinematics::inverse_kinematics(float Vx, float Vy, float Wr){
  float V1, V2, V3, V4 = 0;
  V1 = round((-Vx*this->sin_value[0] + Vy*this->cos_value[0] + Wr*abs(this->err_cos_value[0])));
  V2 = round((-Vx*this->sin_value[1] + Vy*this->cos_value[1] + Wr*abs(this->err_cos_value[1])));
  V3 = round((-Vx*this->sin_value[2] + Vy*this->cos_value[2] + Wr*abs(this->err_cos_value[2])));
  V4 = round((-Vx*this->sin_value[3] + Vy*this->cos_value[3] + Wr*abs(this->err_cos_value[3])));
  //Results is in m/s (linear velocity in X, Y and linear rotation velocity)
  Vwheel[0] = V1; Vwheel[1] = V2; Vwheel[2] = V3; Vwheel[3] = V4;
}

// Get the coordinate position of the robot in X, Y and psi (heading), and robot's velocity relative to global coordinate
void Kinematics::forward_kinematics(float v1, float v2, float v3, float v4, bool odom){
  float x_, y_, w_ = 0;
  float w1 = 0.1229; float w2 = 1.0465; float w3=1.8785; float w4 = 0.9521;
  if(odom){
    float err_v1 = v1-this->prev_v1; float err_v2 = v2-this->prev_v2; float err_v3 = v3-this->prev_v3; float err_v4 = v4-this->prev_v4; 
    x_ = (w1*err_v1*(-this->sin_value[0]) + w2*err_v2*(-this->sin_value[1]) + w3*err_v3*(-this->sin_value[2]) + w4*err_v4*(-this->sin_value[3]))/2;
    y_ = (w1*(err_v1*(this->cos_value[0])) + w2*(err_v2*(this->cos_value[1])) + w3*(err_v3*(this->cos_value[2])) + w4*(err_v4*(this->cos_value[3])))/2;
    w_ = (err_v1*abs(this->err_cos_value[0])+err_v2*abs(this->err_cos_value[1])+err_v3*abs(this->err_cos_value[2])+err_v4*abs(this->err_cos_value[3]))/4;
    this->dist_travel[0] += x_*10*PI/PPR; this->dist_travel[1] += y_*10*PI/PPR; this->dist_travel[2] += (w_*10/PPR)/47.1699*360;
    this->prev_v1 = v1; this->prev_v2 = v2; this->prev_v3 = v3; this->prev_v4 = v4;
  } else{
    x_ = ((w1*v1*(-this->sin_value[0])) + (w2*v2*(-this->sin_value[1])) + (w3*v3*(-this->sin_value[2])) + (w4*v4*(-this->sin_value[3])))/2;
    y_ = ((w1*v1*(this->cos_value[0])) + (w2*v2*(this->cos_value[1])) + (w3*v3*(this->cos_value[2])) + (w4*v4*(this->cos_value[3])))/2;
    w_ = (v1*abs(this->err_cos_value[0])+v2*abs(this->err_cos_value[1])+v3*abs(this->err_cos_value[2])+v4*abs(this->err_cos_value[3]))/4;
    Vreal[0] = x_*10*PI/60; Vreal[1] = y_*10*PI/60; Vreal[2] = (w_*10/60)/47.1699*360;
  }
}
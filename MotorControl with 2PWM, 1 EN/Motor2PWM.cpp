#include "Motor2PWM.h"

Motor2PWM::Motor2PWM(){
}

Motor2PWM::Motor2PWM(byte RPWM, byte LPWM){
  this->RPWM = RPWM;
  this->LPWM = LPWM;

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
}

void Motor2PWM::cw(float pwm){
  analogWrite(RPWM, pwm);
  analogWrite(LPWM, 0);
}

void Motor2PWM::ccw(float pwm){
  analogWrite(RPWM, 0);
  analogWrite(LPWM, pwm);
}

void Motor2PWM::off(){
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}
#include "Motor.h"

//Constructor
Motor::Motor(){
  return;
}

Motor::Motor(byte pin_selector, byte pin_pwm) {
  //Pin mode
  pinMode(pin_selector, OUTPUT);
  pinMode(pin_pwm, OUTPUT);

  //Arrange the private variable object
  this->pin_selector = pin_selector;
  this->pin_pwm = pin_pwm;
}

/*MOTOR MOVEMENT*/
void Motor::cw(byte speed){
  digitalWrite(pin_selector, LOW);
  analogWrite(pin_pwm, speed);
}

void Motor::ccw(byte speed){
  digitalWrite(pin_selector, HIGH);
  analogWrite(pin_pwm, speed);
}

void Motor::stop(){
  analogWrite(pin_pwm, 0);
}

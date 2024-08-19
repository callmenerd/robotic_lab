#include "Lifter.h"

Lifter::Lifter(){
  ;
}

Lifter::Lifter(byte slider_rpwm, byte slider_lpwm, byte roller_rpwm, byte roller_lpwm){
  this->motor_slider = Motor2PWM(slider_rpwm, slider_lpwm);
  this->motor_roller = Motor2PWM(roller_rpwm, roller_lpwm);
}

void Lifter::system_off(){
  this->motor_roller.off();
  this->motor_slider.off();
}

void Lifter::set_motor_pwm(byte slider, byte roller){
  this->_pwm_roller = roller;
  this->_pwm_slider = slider;
}

void Lifter::do_intake(){
  this->motor_roller.cw(this->_pwm_roller);
}

void Lifter::do_outtake(){
  this->motor_roller.ccw(this->_pwm_roller);
}

void Lifter::do_up_lift(){
  this->motor_slider.cw(this->_pwm_slider);
}

void Lifter::intake_and_lift(){
  this->motor_roller.cw(this->_pwm_roller);
  this->motor_slider.cw(this->_pwm_slider);
}

void Lifter::lift_off(){
  this->motor_slider.cw(this->_pwm_slider);
}

void Lifter::in_out_take_off(){
  this->motor_roller.cw(this->_pwm_roller);
}
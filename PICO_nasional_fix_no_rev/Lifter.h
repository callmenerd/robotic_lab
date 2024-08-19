#ifndef Lifter_H
#define Lifter_H

#include <Arduino.h>
#include "Motor2PWM.h"

class Lifter{
  private:
    //PIN AND OBJECT
    byte slider, roller;
    Motor2PWM motor_roller;
    Motor2PWM motor_slider;

    //Variable
    byte _pwm_roller = 220;
    byte _pwm_slider = 150;
    bool _prev_ir_in = false;
    bool _prev_ir_out = false;

  public:
    byte sequence = 0;
    byte color_ball = 0;
    unsigned long count = 0;

    bool ir_in = false;
    bool ir_out = false;
    bool isActive = false;
    char feedback_to_pi = ' ';

    Lifter();
    Lifter(byte slider_rpwm, byte slider_lpwm, byte roller_rpwm, byte roller_lpwm);
    //Setting variable
    void set_motor_pwm(byte slider, byte roller);
    void update_ir(int in, int out);

    //Actuator method
    void intake_and_lift();
    void do_intake();
    void do_outtake();
    void do_up_lift();
    void lift_off();
    void in_out_take_off();
    void system_off();
};

#endif
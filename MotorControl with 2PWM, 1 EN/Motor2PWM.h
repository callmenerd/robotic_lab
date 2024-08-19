#ifndef Motor2PWM_H
#define Motor2PWM_H

#include <Arduino.h>

class Motor2PWM{
  private:
    byte RPWM;
    byte LPWM;
  public:
    Motor2PWM();
    Motor2PWM(byte RPWM, byte LPWM);
    void cw(float pwm);
    void ccw(float pwm);
    void off();
};

#endif
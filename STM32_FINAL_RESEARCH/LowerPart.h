#ifndef LOWER_H
#define LOWER_H

#include "Motor.h"

class LowerPart{
  private:
    // Object for every motor
    Motor roda_1;
    Motor roda_2;
    Motor roda_3;
    Motor roda_4;

  public:
    LowerPart();
    LowerPart(byte sel_1, byte pwm_1, byte sel_2, byte pwm_2, byte sel_3, byte pwm_3, byte sel_4, byte pwm_4);

    void Movement(float V1, float V2, float V3, float V4);
};

#endif
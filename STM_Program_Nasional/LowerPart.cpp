#include "LowerPart.h"

/* KONSTRUKTOR */
LowerPart::LowerPart(){
  return;
}

LowerPart::LowerPart(byte sel_1, byte pwm_1, byte sel_2, byte pwm_2, byte sel_3, byte pwm_3, byte sel_4, byte pwm_4){  
  roda_1 = Motor(sel_1, pwm_1);
  roda_2 = Motor(sel_2, pwm_2);
  roda_3 = Motor(sel_3, pwm_3);
  roda_4 = Motor(sel_4, pwm_4);
}

void LowerPart::Movement(float V1, float V2, float V3, float V4){
  if(V1 > 0){
    roda_1.ccw(abs(V1));
  } else if(V1 < 0){
    roda_1.cw(abs(V1));
  } else{
    roda_1.stop();
  }

  if(V2 > 0){
    roda_2.ccw(abs(V2));
  } else if(V2 < 0){
    roda_2.cw(abs(V2));
  } else{
    roda_2.stop();
  }

  if(V3 > 0){
    roda_3.ccw(abs(V3));
  } else if(V3 < 0){
    roda_3.cw(abs(V3));
  } else{
    roda_3.stop();
  }

  if(V4 > 0){
    roda_4.ccw(abs(V4));
  } else if(V4 < 0){
    roda_4.cw(abs(V4));
  } else{
    roda_4.stop();
  }
}
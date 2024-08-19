/*----------------------------------------------
  ---------PROPERTY OF BLAKASUTHA_UNSOED--------
  ----------------------------------------------
*/
#ifndef KINEMATICS_H
#define KINEMATICS_H

#include<Arduino.h>
#include "SpeedTrig.h"

class Kinematics{
  private:
    float a1; float a2; float a3; float a4; //Angle of the wheel measure from the center of the robot
    float a5; float a6;
    float r, R; //radius of the wheel and radius between the wheel and center point
  public:
    /* ---- PUBLIC VARIABLE ---- */
    float Vw[4] = {0,0,0,0}; //Each wheel velocity from IK
    float Vr[3] = {0,0,0}; //Real Velocity from FK
    float s[3] = {0,0,0}; //Real distance travel from FK
    float S_od[3] = {0,0,0};
    float PPR = 537.6; //default PPR
    float PPR2 = 2400;


    /* ---- METHOD ---- */
    Kinematics();
    Kinematics(float a1, float a2, float a3, float a4, float a5, float a6, float r, float R);
    void forward_kin(float v1, float v2, float v3, float v4, float xi=0, bool odom = false);
    void inverse_kin(float Vx, float Vy, float Wr, float xi=0);
    void forward_pulse(float p1, float p2, float xi=0);
    
};
#endif

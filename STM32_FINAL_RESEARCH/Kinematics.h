/*----------------------------------------------
  ---------PROPERTY OF BLAKASUTHA_UNSOED--------
  ----------------------------------------------
*/
#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>
#include <FastTrig.h>

class Kinematics{
  private:
    float ideal_a1; float ideal_a2; float ideal_a3; float ideal_a4; //Angle of the wheel measure from the center of the robot
    float a1; float a2; float a3; float a4; //Angle of the wheel measure from the center of the robot
    float r, R; //radius of the wheel and radius between the wheel and center point
    float sin_value[4] = {0,0,0,0}; float cos_value[4] = {0,0,0,0}; float err_cos_value[4] = {0,0,0,0};
    float prev_v1, prev_v2, prev_v3, prev_v4 = 0;

  public:
    /* ---- PUBLIC VARIABLE ---- */
    float Vwheel[4] = {0,0,0,0}; //Each wheel velocity from IK
    float Vreal[3] = {0,0,0}; //Real Velocity from FK
    float dist_travel[3] = {0,0,0}; //Real distance travel from FK
    float PPR = 537.6; //default PPR

    /* ---- METHOD ---- */
    Kinematics();
    Kinematics(float a1, float a2, float a3, float a4, float r, float R);
    void init(void);
    void set_ideal_value(float i_a1 = 45, float i_a2 = 135, float i_a3 = 225, float i_a4 = 315);
    void update_angle(float psi = 0);
    void forward_kinematics(float v1, float v2, float v3, float v4, bool odom = false);
    void inverse_kinematics(float Vx, float Vy, float Wr);
};
#endif

#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

#define DEBUG true
#define DEBUG_SERIAL if(DEBUG)Serial

class Motor {
    private:
        //Selector and pwm pin for motor;
        byte pin_selector;
        byte pin_pwm;
        
    public:
        //Contructor
        Motor();
        Motor(byte pin_selector, byte pin_pwm);

        //Control of motor rotation
        void cw(byte speed);
        void ccw(byte speed);
        void stop();
};

#endif

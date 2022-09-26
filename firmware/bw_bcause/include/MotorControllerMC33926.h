#pragma once
#include <Arduino.h>


class MotorControllerMC33926
{
private:
    int SPEED;
    int DIR_P;
    int DIR_N;
    int SF;
    int FB;


public:
    MotorControllerMC33926(int speed_pin, int dir_p_pin, int dir_n_pin, int sf_pin, int fb_pin);
    
    void set(int speed);
    void begin();

    bool read_status();
    int read_feedback();
};

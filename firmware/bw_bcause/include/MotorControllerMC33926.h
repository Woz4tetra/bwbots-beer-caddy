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
    int frequency;


public:
    MotorControllerMC33926(int speed_pin, int dir_p_pin, int dir_n_pin, int sf_pin, int fb_pin, int frequency);
    
    void set(int speed);
    void begin();

    void set_frequency(int frequency);

    bool read_status();
    int read_feedback();
};

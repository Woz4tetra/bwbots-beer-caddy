#pragma once
#include <Arduino.h>

class SpeedFilter
{
private:
    double accum_value;
    double prev_value;
    uint32_t prev_time;
    double prev_velocity;
public:
    double Kf;
    SpeedFilter(double Kf);
    void reset();
    double compute(double next_value);
    double get_velocity();
    double dt();
};

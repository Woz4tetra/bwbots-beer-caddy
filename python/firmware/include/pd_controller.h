#pragma once

#include <climits>
#include <Arduino.h>

class PDController
{
private:
    double kp, kd;
    double prev_error;
    double tolerance;
    uint32_t prev_time;

    double dt();
public:
    PDController(double kp, double kd, double tolerance);
    void reset();
    double update(double error);

    void set_kp(double kp)  { this->kp = kp; }
    double get_kp()  { return this->kp; }

    void set_kd(double kd)  { this->kd = kd; }
    double get_kd()  { return this->kd; }
};

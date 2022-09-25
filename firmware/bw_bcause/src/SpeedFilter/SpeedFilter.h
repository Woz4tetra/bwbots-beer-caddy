#include <Arduino.h>

class SpeedFilter
{
private:
    double accum_value;
    double prev_value;
    uint32_t prev_time;
public:
    double Kf;
    SpeedFilter(double Kf);
    double compute(double next_value);
    double dt();
};

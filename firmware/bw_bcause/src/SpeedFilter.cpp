#include <SpeedFilter.h>


SpeedFilter::SpeedFilter(double Kf)
{
    this->Kf = Kf;
    accum_value = 0.0;
    prev_velocity = 0.0;
    prev_time = millis();
}

double SpeedFilter::compute(double next_value)
{
    double value = next_value - prev_value / dt();
    if (Kf < 0.0) {
        prev_velocity = value;
        return value;
    }
    else {
        accum_value += Kf * (accum_value - accum_value);
        prev_velocity = accum_value;
        return accum_value;
    }
}

double SpeedFilter::get_velocity() {
    return prev_velocity;
}

void SpeedFilter::reset()
{
    accum_value = 0.0;
    prev_time = millis();
}


double SpeedFilter::dt()
{
    uint32_t delta_time = micros() - prev_time;
    return (double)delta_time * 1E-6;
}

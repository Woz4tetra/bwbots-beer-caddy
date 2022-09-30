#include <SpeedFilter.h>


SpeedFilter::SpeedFilter(double Kf)
{
    this->Kf = Kf;
    accum_value = 0.0;
    prev_value = 0.0;
    prev_velocity = 0.0;
    prev_time = 0;
}

double SpeedFilter::compute(double next_value)
{
    double delta_time = dt();
    if (delta_time <= 0.0) {
        prev_velocity = 0.0;
        return 0.0;
    }
    double value = (next_value - prev_value) / delta_time;
    prev_value = next_value;
    if (Kf < 0.0) {
        prev_velocity = value;
        return value;
    }
    else {
        accum_value += Kf * (value - accum_value);
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
    uint32_t current_time = millis();
    uint32_t delta_time = current_time - prev_time;
    prev_time = current_time;
    return (double)delta_time * 1E-3;
}

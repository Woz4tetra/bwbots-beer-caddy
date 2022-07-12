#include "pd_controller.h"


PDController::PDController(double kp, double kd, double tolerance)
{
    this->kp = kp;
    this->kd = kd;
    this->tolerance = tolerance;
    prev_error = 0.0;
}

void PDController::reset()
{
    prev_error = 0.0;
}

double PDController::update(double error)
{
    if (error < tolerance) {
        return 0.0;
    }
    return kp * error + kd * (error - prev_error) / dt();
}

double PDController::dt()
{
    uint32_t current_time = millis();
    uint32_t delta;
    if (current_time < prev_time) {
        delta = ULONG_MAX - prev_time + current_time;
    }
    else {
        delta = current_time - prev_time;
    }
    double seconds = (double)(delta) * 1E-3;
    prev_time = current_time;
    return seconds;
}

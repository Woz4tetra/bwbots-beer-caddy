#include <SpeedPID.h>


#define DEBUG_SERIAL Serial2


SpeedPID::SpeedPID()
{
    target = 0.0;
    error_sum = 0.0;
    prev_error = 0.0;
    feedforward = 0.0;
    current_time = 0;
    prev_update_time = 0;
    dt = 0.0;
    out = 0.0;
    command_min = -255;
    command_max = 255;
    epsilon = 1E-3;

    K_ff = 0.0;
    deadzone_command = 0.0;
    standstill_deadzone_command = 0.0;
    error_sum_clamp = 0.0;
    Kp = 1.0;
    Ki = 0.0;
    Kd = 0.0;
}

double SpeedPID::sign(double x) {
    return (x > 0) - (x < 0);
}

int SpeedPID::sign(int x) {
    return (x > 0) - (x < 0);
}


void SpeedPID::set_target(double target) {
    feedforward = K_ff * target;
    if (sign(target) != this->target) {
        error_sum = 0.0;
    }
    this->target = target;
    prev_update_time = 0;
}

double SpeedPID::get_target() {
    return target;
}

void SpeedPID::reset() {
    prev_error = 0.0;
    error_sum = 0.0;
    set_target(0.0);
}

double SpeedPID::limit(double command) {
    if (abs(command) < epsilon) {
        return 0.0;
    }

    if (command > command_max) {
        return command_max;
    }
    if (command < command_min) {
        return command_min;
    }

    if (abs(command) < deadzone_command) {
        return sign(command) * deadzone_command;
    }

    return command;
}

double SpeedPID::compute(double measurement)
{
    current_time = millis();

    if (current_time - prev_update_time == 0) {
        DEBUG_SERIAL.println("PID timer didn't change!");
        return limit(out);
    }
    else if (current_time - prev_update_time < 0) {  // edge case for timer looping
        DEBUG_SERIAL.println("PID timer overflowed! Resetting.");
        prev_update_time = 0;
    }

    double error = target - measurement;
    dt = (current_time - prev_update_time) * 1E-3;
    prev_update_time = current_time;

    out = 0.0;
    if (Kp > 0.0) {
        out += Kp * error;
    }
    if (Kd > 0.0) {
        out += Kd * (error - prev_error) / dt;
        prev_error = error;
    }
    if (Ki > 0.0) {
        out += Ki * error_sum * dt;
        error_sum += error;
        if (error_sum_clamp >= 0.0 && abs(error_sum) > error_sum_clamp) {
            error_sum = sign(error_sum) * error_sum_clamp;
        }
    }
    out += feedforward;

    if (standstill_deadzone_command != 0.0 && abs(target) > epsilon && abs(measurement) < epsilon) {
        out = sign(target) * standstill_deadzone_command;
    }

    return limit(out);
}

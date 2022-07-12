#include "balance_controller.h"

BalanceController::BalanceController(Chassis* chassis, Adafruit_BNO055* bno, double setpoint, double kp, double kd, double tolerance)
{
    this->chassis = chassis;
    this->bno = bno;
    this->controller = new PDController(kp, kd, tolerance);
    report_timer = 0;
    sample_delay = 33;
    angle_setpoint = setpoint;
    enabled = false;
}

void BalanceController::reset()
{
    controller->reset();
}

bool BalanceController::update()
{
    // uint32_t current_time = millis();
    bno->getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    if (enabled)
    {
        double voltage = controller->update(get_imu_y() - angle_setpoint);
        chassis->set_left_motor((int)(VOLTS_TO_PWM * voltage));
        chassis->set_right_motor((int)(VOLTS_TO_PWM * voltage));
    }

    uint32_t current_time = millis();
    if (current_time - report_timer < sample_delay) {
        return false;
    }
    if (current_time < report_timer) {
        report_timer = current_time;
        return false;
    }
    report_timer = current_time;
    return true;
}


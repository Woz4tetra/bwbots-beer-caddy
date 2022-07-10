#include "balance_controller.h"

BalanceController::BalanceController(Chassis* chassis, Adafruit_BNO055* bno)
{
    this->chassis = chassis;
    this->bno = bno;
    report_timer = 0;
    sample_delay = 33;
    angle_setpoint = 0.0f;
}

void BalanceController::begin()
{

}

bool BalanceController::update()
{
    // uint32_t current_time = millis();
    bno->getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

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


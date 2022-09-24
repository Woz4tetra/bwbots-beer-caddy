#include <MotorControllerMC33926/MotorControllerMC33926.h>


MotorControllerMC33926::MotorControllerMC33926(int speed_pin, int dir_p_pin, int dir_n_pin, int sf_pin, int fb_pin)
{
    SPEED = speed_pin;
    DIR_P = dir_p_pin;
    DIR_N = dir_n_pin;
    SF = sf_pin;
    FB = fb_pin;
}

void MotorControllerMC33926::begin()
{
    pinMode(SPEED, OUTPUT);
    pinMode(DIR_P, OUTPUT);
    pinMode(DIR_N, OUTPUT);
    pinMode(SF, INPUT);
    pinMode(FB, INPUT);
    analogWriteFrequency(SPEED, 500);  // Set PWM frequency of the speed pin to 500 Hz
}

void MotorControllerMC33926::set(int speed)
{
    if (speed == 0) {
        digitalWrite(DIR_P, LOW);
        digitalWrite(DIR_N, LOW);
    }
    else if (speed > 0) {
        digitalWrite(DIR_P, HIGH);
        digitalWrite(DIR_N, LOW);
    }
    else {
        digitalWrite(DIR_P, LOW);
        digitalWrite(DIR_N, HIGH);
    }
    analogWrite(SPEED, abs(speed));
}

bool MotorControllerMC33926::read_status() {
    return (bool)digitalRead(SF);
}

int MotorControllerMC33926::read_feedback() {
    return analogRead(FB);
}

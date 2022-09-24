#include <Encoder.h>
#include <Adafruit_PWMServoDriver.h>
#include <MotorControllerMC33926/MotorControllerMC33926.h>

class BwDriveTrain
{
private:
    Adafruit_PWMServoDriver* servos;
    MotorControllerMC33926** motors;
    Encoder** encoders;
    unsigned int num_motors;
    unsigned int motor_enable_pin;
    static const unsigned int MAX_CHANNELS = 16;

public:
    BwDriveTrain(
        Adafruit_PWMServoDriver* servos,
        MotorControllerMC33926** motors,
        Encoder** encoders,
        unsigned int num_motors,
        unsigned int motor_enable_pin
    );
    void begin();
    void drive(float vx, float vy, float vt);
};

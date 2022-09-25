#include <Encoder.h>
#include <Adafruit_PWMServoDriver.h>
#include <MotorControllerMC33926/MotorControllerMC33926.h>
#include <SpeedPID/SpeedPID.h>
#include <SpeedFilter/SpeedFilter.h>

class BwDriveTrain
{
private:
    Adafruit_PWMServoDriver* servos;
    MotorControllerMC33926** motors;
    Encoder** encoders;
    long* encoder_positions;
    SpeedPID** speed_pids;
    SpeedFilter** speed_filters;
    unsigned int num_motors;
    unsigned int motor_enable_pin;
    bool is_enabled;
    static const unsigned int MAX_CHANNELS = 16;
    double output_ratio;  // encoder counts * output_ratio = m/s at wheel

public:
    BwDriveTrain(
        Adafruit_PWMServoDriver* servos,
        MotorControllerMC33926** motors,
        Encoder** encoders,
        unsigned int num_motors,
        unsigned int motor_enable_pin,
        double output_ratio
    );
    void begin();
    void set_enable(bool state);
    bool get_enable();
    void drive(float vx, float vy, float vt);
    SpeedPID* get_pid(unsigned int channel);
    SpeedFilter* get_filter(unsigned int channel);
    double get_velocity(int channel);
    double get_position(int channel);
};

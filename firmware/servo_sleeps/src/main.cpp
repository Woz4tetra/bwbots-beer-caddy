#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define I2C_BUS_1 Wire


const int STATUS_LED = 0;
const int BUTTON_LED = 14;
const int BUILTIN_LED = 13;
const int MOTOR_EN = 1;

const int BUTTON_IN = 15;


const int SERVOMIN = 150; // This is the 'minimum' pulse length count (out of 4096)
const int SERVOMAX = 600; // This is the 'maximum' pulse length count (out of 4096)
const int SERVO_FREQ = 50; // Analog servos run at ~50 Hz updates
Adafruit_PWMServoDriver* servos = new Adafruit_PWMServoDriver(0x40 + 0b000010, I2C_BUS_1);


bool read_button() {
    return !digitalRead(BUTTON_IN);
}

bool prev_button_state = false;
bool did_button_press(bool comparison_state) {
    bool state = read_button();
    if (state != prev_button_state) {
        prev_button_state = state;
        return state == comparison_state;
    }
    else {
        return false;
    }
}

void set_status_led(bool state) {
    digitalWrite(STATUS_LED, state);
}

void set_builtin_led(bool state) {
    digitalWrite(BUILTIN_LED, state);
}

void set_button_led(bool state) {
    digitalWrite(BUTTON_LED, state);
}

void setup()
{
    Serial.begin(9600);
    I2C_BUS_1.begin();
    I2C_BUS_1.setSDA(18);
    I2C_BUS_1.setSCL(19);

    servos->begin();
    servos->setOscillatorFrequency(27000000);
    servos->setPWMFreq(SERVO_FREQ);

    pinMode(STATUS_LED, OUTPUT);
    pinMode(MOTOR_EN, OUTPUT);

    pinMode(BUTTON_IN, INPUT_PULLUP);
    pinMode(BUTTON_LED, OUTPUT);
    pinMode(BUILTIN_LED, OUTPUT);

    set_status_led(true);
    set_builtin_led(true);
    set_button_led(true);
}

bool enabled = false;

void loop()
{
    if (did_button_press(true)) {
        enabled = !enabled;
        Serial.println(enabled);
        if (enabled) {
            servos->setPWM(0, 0, 450);
            servos->setPWM(1, 0, 450);
            servos->setPWM(2, 0, 450);
            servos->setPWM(3, 0, 450);
        }
        else {
            
            servos->setPWM(0, 0, 4096);
            servos->setPWM(1, 0, 4096);
            servos->setPWM(2, 0, 4096);
            servos->setPWM(3, 0, 4096);
        }
    }
}

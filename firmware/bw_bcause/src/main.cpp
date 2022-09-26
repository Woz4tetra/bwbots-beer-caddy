#include <Arduino.h>
#include <Wire.h>
#include <Encoder.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_INA219.h>
#include <Adafruit_NeoPixel.h>
#include <MotorControllerMC33926.h>
#include <BwDriveTrain.h>

#define I2C_BUS_1 Wire


// ---
// Status LEDs
// ---
const int STATUS_LED = 0;
const int BUTTON_LED = 14;
const int BUILTIN_LED = 13;

// ---
// Buttons
// ---

const int BUTTON_IN = 15;

// ---
// Drive controllers
// ---
const int MOTOR_EN = 1;
const int NUM_CHANNELS = 4;

const int M1_SPEED = 9;
const int M1_DIR_P = 11;
const int M1_DIR_N = 12;
const int M1_SF = 32;
const int M1_FB = 23;

const int M2_SPEED = 10;
const int M2_DIR_P = 28;
const int M2_DIR_N = 29;
const int M2_SF = 17;
const int M2_FB = 22;

const int M3_SPEED = 37;
const int M3_DIR_P = 30;
const int M3_DIR_N = 31;
const int M3_SF = 38;
const int M3_FB = 21;

const int M4_SPEED = 36;
const int M4_DIR_P = 35;
const int M4_DIR_N = 34;
const int M4_SF = 33;
const int M4_FB = 20;

MotorControllerMC33926* motors[NUM_CHANNELS] = {
    new MotorControllerMC33926(M1_SPEED, M1_DIR_P, M1_DIR_N, M1_SF, M1_FB),
    new MotorControllerMC33926(M2_SPEED, M2_DIR_P, M2_DIR_N, M2_SF, M2_FB),
    new MotorControllerMC33926(M3_SPEED, M3_DIR_P, M3_DIR_N, M3_SF, M3_FB),
    new MotorControllerMC33926(M4_SPEED, M4_DIR_P, M4_DIR_N, M4_SF, M4_FB)
};

const int MOTOR1_ENCA = 3;
const int MOTOR1_ENCB = 4;
const int MOTOR2_ENCA = 5;
const int MOTOR2_ENCB = 6;
const int MOTOR3_ENCA = 24;
const int MOTOR3_ENCB = 25;
const int MOTOR4_ENCA = 26;
const int MOTOR4_ENCB = 27;

Encoder* encoders[NUM_CHANNELS] = {
    new Encoder(MOTOR1_ENCA, MOTOR1_ENCB),
    new Encoder(MOTOR2_ENCA, MOTOR2_ENCB),
    new Encoder(MOTOR3_ENCA, MOTOR3_ENCB),
    new Encoder(MOTOR4_ENCA, MOTOR4_ENCB)
};

// ---
// Servos
// ---

const int SERVOMIN = 150; // This is the 'minimum' pulse length count (out of 4096)
const int SERVOMAX = 600; // This is the 'maximum' pulse length count (out of 4096)
const int SERVO_FREQ = 50; // Analog servos run at ~50 Hz updates
const int SERVO_OSC_FREQ = 27000000;
Adafruit_PWMServoDriver* servos = new Adafruit_PWMServoDriver(0x40 + 0b000010, I2C_BUS_1);

// ---
// Drive train
// ---
const double MAX_MOTOR_SPEED = 0.843;  // m/s
const double MAX_SERVO_SPEED = 5.950;  // rad/s
const int DEADZONE_COMMAND = 50;
const int MAX_SPEED_COMMAND = 255;
const double ALCOVE_ANGLE = 0.5236;  // 30 degrees
const double FRONT_ANGLE = -1.2967;  // -74.293 degrees
const double STRAIGHT_ANGLE = 0.0;  // 0 degrees
const double GEAR_RATIO = 54.0;
const double ENCODER_PPR = 11.0;  // pulses per rotation
const double WHEEL_DIAMETER = 0.115;  // meters
const double OUTPUT_RATIO = 2 * M_PI * WHEEL_DIAMETER / (GEAR_RATIO * ENCODER_PPR);  // encoder counts (pulses) * output_ratio = m/s at wheel
BwDriveTrain drive(servos, motors, encoders, NUM_CHANNELS, MOTOR_EN, OUTPUT_RATIO);

// ---
// Power management
// ---

Adafruit_INA219 charge_ina(0x40 + 0b01);

// ---
// LED ring
// ---

const int LED_RING = 39;
const int NUM_PIXELS = 24;
Adafruit_NeoPixel led_ring(NUM_PIXELS, LED_RING, NEO_GRBW + NEO_KHZ800);


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
    servos->setOscillatorFrequency(SERVO_OSC_FREQ);
    servos->setPWMFreq(SERVO_FREQ);

    charge_ina.begin();

    led_ring.begin();

    pinMode(STATUS_LED, OUTPUT);
    pinMode(MOTOR_EN, OUTPUT);

    pinMode(BUTTON_IN, INPUT_PULLUP);
    pinMode(BUTTON_LED, OUTPUT);
    pinMode(BUILTIN_LED, OUTPUT);

    set_status_led(true);
    set_builtin_led(true);
    set_button_led(true);

    for (unsigned int channel = 0; channel < drive.get_num_motors(); channel++) {
        SpeedPID* pid = drive.get_pid(channel);
        pid->Kp = 1.0;
        pid->Ki = 0.0;
        pid->Kd = 0.0;
        pid->K_ff = (double)MAX_SPEED_COMMAND / MAX_MOTOR_SPEED;
        pid->deadzone_command = DEADZONE_COMMAND;
        pid->error_sum_clamp = -1.0;
        pid->command_min = -MAX_SPEED_COMMAND;
        pid->command_max = MAX_SPEED_COMMAND;
        pid->command_timeout_ms = 1000;
    }
    
    drive.set_limits(
        0,
        -ALCOVE_ANGLE,
        -FRONT_ANGLE,
        STRAIGHT_ANGLE,
        -ALCOVE_ANGLE,
        375,
        470,
        MAX_SERVO_SPEED
    );
    drive.set_limits(
        1,
        -ALCOVE_ANGLE,
        -FRONT_ANGLE,
        STRAIGHT_ANGLE,
        -ALCOVE_ANGLE,
        235,
        125,
        MAX_SERVO_SPEED
    );
    drive.set_limits(
        2,
        -ALCOVE_ANGLE,
        -FRONT_ANGLE,
        STRAIGHT_ANGLE,
        -ALCOVE_ANGLE,
        230,
        140,
        MAX_SERVO_SPEED
    );
    drive.set_limits(
        3,
        -ALCOVE_ANGLE,
        -FRONT_ANGLE,
        STRAIGHT_ANGLE,
        -ALCOVE_ANGLE,
        405,
        515,
        MAX_SERVO_SPEED
    );

    drive.begin();

}

int state = 0;
bool enabled = false;

void loop()
{
    if (did_button_press(true)) {
        state = (state + 1) % 4;
        Serial.println(state);
    }
    switch (state)
    {
    case 0:
        drive.set_enable(false);
        break;
    case 1:
        drive.set_enable(true);
        drive.set(0, 0.0, 0.0);
        drive.set(1, 0.0, 0.0);
        drive.set(2, 0.0, 0.0);
        drive.set(3, 0.0, 0.0);
        break;
    case 2:
        drive.set(0, 0.5, 0.1);
        drive.set(1, 0.5, 0.1);
        drive.set(2, 0.5, 0.1);
        drive.set(3, 0.5, 0.1);
        break;
    case 3:
        drive.set(0, -0.5, -0.1);
        drive.set(1, -0.5, -0.1);
        drive.set(2, -0.5, -0.1);
        drive.set(3, -0.5, -0.1);
        break;
    
    default:
        break;
    }

}

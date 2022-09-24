#include <Arduino.h>
#include <Wire.h>
#include <Encoder.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_INA219.h>
#include <Adafruit_NeoPixel.h>
#include <MotorControllerMC33926/MotorControllerMC33926.h>
#include <BwDriveTrain/BwDriveTrain.h>

#define I2C_BUS_1 Wire


// ---
// Status LEDs
// ---
const int STATUS_LED = 0;
const int BUTTON_IN = 15;
const int BUTTON_LED = 14;
const int BUILTIN_LED = 13;

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
}

// ---
// Servos
// ---

const int SERVOMIN = 150; // This is the 'minimum' pulse length count (out of 4096)
const int SERVOMAX = 600; // This is the 'maximum' pulse length count (out of 4096)
const int SERVO_FREQ = 50; // Analog servos run at ~50 Hz updates
Adafruit_PWMServoDriver* servos = new Adafruit_PWMServoDriver(0x40 + 0b000010, I2C_BUS_1);

// ---
// Drive train
// ---
BwDriveTrain drive(servos, motors, encoders, NUM_CHANNELS);

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


void setup()
{
    I2C_BUS_1.begin();
    I2C_BUS_1.setSDA(18);
    I2C_BUS_1.setSCL(19);

    servos->begin();
    servos->setOscillatorFrequency(27000000);
    servos->setPWMFreq(SERVO_FREQ);

    charge_ina.begin();

    led_ring.begin();

    pinMode(STATUS_LED, OUTPUT);
    pinMode(MOTOR_EN, OUTPUT);

    pinMode(BUTTON_IN, INPUT_PULLUP);
    pinMode(BUTTON_LED, OUTPUT);
    pinMode(BUILTIN_LED, OUTPUT);

    drive.begin();
}

void loop()
{
    
}

#include <Arduino.h>
#include <Wire.h>
#include <Encoder.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_INA219.h>
#include <Adafruit_NeoPixel.h>
#include <MotorControllerMC33926/MotorControllerMC33926.h>

#define I2C_BUS_1 Wire

const int STATUS_LED = 0;
const int MOTOR_EN = 1;

const int BUTTON_IN = 15;
const int BUTTON_LED = 14;
const int BUILTIN_LED = 13;


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

MotorControllerMC33926 motor1(M1_SPEED, M1_DIR_P, M1_DIR_N, M1_SF, M1_FB);
MotorControllerMC33926 motor2(M2_SPEED, M2_DIR_P, M2_DIR_N, M2_SF, M2_FB);
MotorControllerMC33926 motor3(M3_SPEED, M3_DIR_P, M3_DIR_N, M3_SF, M3_FB);
MotorControllerMC33926 motor4(M4_SPEED, M4_DIR_P, M4_DIR_N, M4_SF, M4_FB);

const int MOTOR1_ENCA = 3;
const int MOTOR1_ENCB = 4;
const int MOTOR2_ENCA = 5;
const int MOTOR2_ENCB = 6;
const int MOTOR3_ENCA = 24;
const int MOTOR3_ENCB = 25;
const int MOTOR4_ENCA = 26;
const int MOTOR4_ENCB = 27;

Encoder enc1(MOTOR1_ENCA, MOTOR1_ENCB);
Encoder enc2(MOTOR2_ENCA, MOTOR2_ENCB);
Encoder enc3(MOTOR3_ENCA, MOTOR3_ENCB);
Encoder enc4(MOTOR4_ENCA, MOTOR4_ENCB);

const int SERVOMIN = 150; // This is the 'minimum' pulse length count (out of 4096)
const int SERVOMAX = 600; // This is the 'maximum' pulse length count (out of 4096)
const int USMIN = 600; // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
const int USMAX = 2400; // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
const int SERVO_FREQ = 50; // Analog servos run at ~50 Hz updates
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40 + 0b000010, I2C_BUS_1);

Adafruit_INA219 charge_ina(0x40 + 0b01);

const int LED_RING = 39;
const int NUM_PIXELS = 24;
Adafruit_NeoPixel led_ring(NUM_PIXELS, LED_RING, NEO_GRBW + NEO_KHZ800);

void report_encoders()
{
    long enc1_value = enc1.read();
    long enc2_value = enc2.read();
    long enc3_value = enc3.read();
    long enc4_value = enc4.read();

    Serial.print(enc1_value);
    Serial.print('\t');
    Serial.print(enc2_value);
    Serial.print('\t');
    Serial.print(enc3_value);
    Serial.print('\t');
    Serial.print(enc4_value);
    Serial.print('\n');
}

void set_status_led(bool state) {
    digitalWrite(STATUS_LED, state);
}

void set_builtin_led(bool state) {
    digitalWrite(BUILTIN_LED, state);
}

void set_motor_enable(bool state) {
    digitalWrite(MOTOR_EN, state);
}

void set_button_led(bool state) {
    digitalWrite(BUTTON_LED, state);
}

bool read_button() {
    return !digitalRead(BUTTON_IN);
}

void setup()
{
    Serial.begin(9600);

    I2C_BUS_1.begin();
    I2C_BUS_1.setSDA(18);
    I2C_BUS_1.setSCL(19);

    servos.begin();
    servos.setOscillatorFrequency(27000000);
    servos.setPWMFreq(SERVO_FREQ);

    charge_ina.begin();

    led_ring.begin();

    pinMode(STATUS_LED, OUTPUT);
    pinMode(MOTOR_EN, OUTPUT);

    pinMode(BUTTON_IN, INPUT_PULLUP);
    pinMode(BUTTON_LED, OUTPUT);
    pinMode(BUILTIN_LED, OUTPUT);

    set_status_led(true);
    set_builtin_led(true);
    set_motor_enable(true);
    set_button_led(true);

    motor1.begin();
    motor2.begin();
    motor3.begin();
    motor4.begin();
    
    enc1.write(0);
    enc2.write(0);
    enc3.write(0);
    enc4.write(0);
}

void set_all_motors(int speed) {
    motor1.set(speed);
    motor2.set(speed);
    motor3.set(speed);
    motor4.set(speed);
}

void set_all_servos(int position) {
    int signal = map(position, -255, 255, SERVOMIN, SERVOMAX);
    servos.setPWM(0, 0, signal);
    servos.setPWM(1, 0, signal);
    servos.setPWM(2, 0, signal);
    servos.setPWM(3, 0, signal);
    // Serial.println(signal);
}

int motor_value = 0;
int base_increment = 1;
int increment = 1;
uint32_t increment_delay = 10;
uint32_t increment_timer = 0;
uint32_t current_time = 0;

void loop()
{
    // set_button_led(read_button());

    if (read_button())
    {
        set_button_led(false);
        set_all_motors(100);
        set_all_servos(50);
        delay(1000);
        for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {
            led_ring.setPixelColor(pixel, led_ring.Color(255, 0, 0, 0));
            delay(20);
            led_ring.show();
        }
        set_all_motors(-100);
        set_all_servos(-50);
        delay(1000);
        for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {
            led_ring.setPixelColor(pixel, led_ring.Color(0, 255, 0, 0));
            delay(20);
            led_ring.show();
        }
        set_all_motors(0);
    }
    else {
        set_button_led(true);
    }

    // current_time = millis();
    // if (current_time - increment_timer > increment_delay) {
    //     motor_value += increment;
    //     Serial.println(motor_value);
    //     if (motor_value >= 255) {
    //         increment = -base_increment;

    //         set_all_servos(50);
    //         for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {
    //             led_ring.setPixelColor(pixel, led_ring.Color(abs(motor_value), 0, 0, 0));
    //         }
    //         led_ring.show();
    //     }
    //     else if (motor_value <= -255) {
    //         increment = base_increment;

    //         set_all_servos(-50);
    //         for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {
    //             led_ring.setPixelColor(pixel, led_ring.Color(0, 0, abs(motor_value), 0));
    //         }
    //         led_ring.show();
    //     }
    //     report_encoders();
    //     set_all_motors(motor_value);
    //     Serial.println(charge_ina.getBusVoltage_V());
    //     increment_timer = current_time;
    // }
}

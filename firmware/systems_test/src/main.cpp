#include <Arduino.h>
#include <Wire.h>
#include <Encoder.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_INA219.h>
#include <MotorControllerMC33926/MotorControllerMC33926.h>

#define I2C_BUS_1 Wire

const int STATUS_LED = 0;
const int MOTOR_EN = 1;

const int BUTTON_IN = 15;
const int BUTTON_LED = 14;


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

Adafruit_INA219 main_ina(0x40);
Adafruit_INA219 charge_ina(0x40 + 0b01);

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

void set_motor_enable(bool state) {
    digitalWrite(MOTOR_EN, state);
}

void set_button_led(bool state) {
    digitalWrite(BUTTON_LED, state);
}

bool read_button() {
    return digitalRead(BUTTON_IN);
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

    main_ina.begin();
    charge_ina.begin();

    pinMode(STATUS_LED, OUTPUT);
    pinMode(MOTOR_EN, OUTPUT);

    pinMode(BUTTON_IN, INPUT_PULLUP);
    pinMode(BUTTON_LED, OUTPUT);

    set_status_led(true);
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

void loop()
{
    Serial.print(main_ina.getPower_mW());
    Serial.print('\t');
    Serial.println(charge_ina.getPower_mW());
    delay(100);

    // Serial.println("150");
    // servos.setPWM(0, 0, 150);
    // servos.setPWM(1, 0, 150);
    // servos.setPWM(2, 0, 150);
    // servos.setPWM(3, 0, 150);

    // delay(1000);

    // Serial.println("600");
    // servos.setPWM(0, 0, 600);
    // servos.setPWM(1, 0, 600);
    // servos.setPWM(2, 0, 600);
    // servos.setPWM(3, 0, 600);

    // delay(1000);

    // for (int value = 0; value < 255; value++) {
    //     motor1.set(value);
    //     motor2.set(value);
    //     motor3.set(value);
    //     motor4.set(value);
    //     delay(50);
    //     report_encoders();
    // }

    // for (int value = 255; value >= -255; value--) {
    //     motor1.set(value);
    //     motor2.set(value);
    //     motor3.set(value);
    //     motor4.set(value);
    //     delay(50);
    //     report_encoders();
    // }

    // for (int value = -255; value < 0; value++) {
    //     motor1.set(value);
    //     motor2.set(value);
    //     motor3.set(value);
    //     motor4.set(value);
    //     delay(50);
    //     report_encoders();
    // }
}

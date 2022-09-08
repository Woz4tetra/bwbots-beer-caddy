#include <Arduino.h>
#include <Encoder.h>

const int STATUS_LED = 0;
const int MOTOR_EN = 1;

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

void set_m1(int speed)
{
    if (speed == 0) {
        digitalWrite(M1_DIR_P, LOW);
        digitalWrite(M1_DIR_N, LOW);
    }
    else if (speed > 0) {
        digitalWrite(M1_DIR_P, HIGH);
        digitalWrite(M1_DIR_N, LOW);
    }
    else {
        digitalWrite(M1_DIR_P, LOW);
        digitalWrite(M1_DIR_N, HIGH);
    }
    analogWrite(M1_SPEED, abs(speed));
    Serial.print("M1 SF: ");
    Serial.print(digitalRead(M1_SF));
    Serial.print("\tFB: ");
    Serial.print(analogRead(M1_FB));
    Serial.print("\tspeed: ");
    Serial.println(speed);
}


void set_m2(int speed)
{
    if (speed == 0) {
        digitalWrite(M2_DIR_P, LOW);
        digitalWrite(M2_DIR_N, LOW);
    }
    else if (speed > 0) {
        digitalWrite(M2_DIR_P, HIGH);
        digitalWrite(M2_DIR_N, LOW);
    }
    else {
        digitalWrite(M2_DIR_P, LOW);
        digitalWrite(M2_DIR_N, HIGH);
    }
    analogWrite(M2_SPEED, abs(speed));
    Serial.print("M2 SF: ");
    Serial.print(digitalRead(M2_SF));
    Serial.print("\tFB: ");
    Serial.print(analogRead(M2_FB));
    Serial.print("\tspeed: ");
    Serial.println(speed);
}

void set_m3(int speed)
{
    if (speed == 0) {
        digitalWrite(M3_DIR_P, LOW);
        digitalWrite(M3_DIR_N, LOW);
    }
    else if (speed > 0) {
        digitalWrite(M3_DIR_P, HIGH);
        digitalWrite(M3_DIR_N, LOW);
    }
    else {
        digitalWrite(M3_DIR_P, LOW);
        digitalWrite(M3_DIR_N, HIGH);
    }
    analogWrite(M3_SPEED, abs(speed));
    Serial.print("M3 SF: ");
    Serial.print(digitalRead(M3_SF));
    Serial.print("\tFB: ");
    Serial.print(analogRead(M3_FB));
    Serial.print("\tspeed: ");
    Serial.println(speed);
}


void set_m4(int speed)
{
    if (speed == 0) {
        digitalWrite(M4_DIR_P, LOW);
        digitalWrite(M4_DIR_N, LOW);
    }
    else if (speed > 0) {
        digitalWrite(M4_DIR_P, HIGH);
        digitalWrite(M4_DIR_N, LOW);
    }
    else {
        digitalWrite(M4_DIR_P, LOW);
        digitalWrite(M4_DIR_N, HIGH);
    }
    analogWrite(M4_SPEED, abs(speed));
    Serial.print("M4 SF: ");
    Serial.print(digitalRead(M4_SF));
    Serial.print("\tFB: ");
    Serial.print(analogRead(M4_FB));
    Serial.print("\tspeed: ");
    Serial.println(speed);
}

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

void setup()
{
    Serial.begin(9600);
    
    enc1.write(0);
    enc2.write(0);
    enc3.write(0);
    enc4.write(0);

    pinMode(STATUS_LED, OUTPUT);
    pinMode(MOTOR_EN, OUTPUT);

    pinMode(M1_SPEED, OUTPUT);
    pinMode(M1_DIR_P, OUTPUT);
    pinMode(M1_DIR_N, OUTPUT);
    pinMode(M1_SF, INPUT);
    pinMode(M1_FB, INPUT);

    pinMode(M2_SPEED, OUTPUT);
    pinMode(M2_DIR_P, OUTPUT);
    pinMode(M2_DIR_N, OUTPUT);
    pinMode(M2_SF, INPUT);
    pinMode(M2_FB, INPUT);

    pinMode(M3_SPEED, OUTPUT);
    pinMode(M3_DIR_P, OUTPUT);
    pinMode(M3_DIR_N, OUTPUT);
    pinMode(M3_SF, INPUT);
    pinMode(M3_FB, INPUT);

    pinMode(M4_SPEED, OUTPUT);
    pinMode(M4_DIR_P, OUTPUT);
    pinMode(M4_DIR_N, OUTPUT);
    pinMode(M4_SF, INPUT);
    pinMode(M4_FB, INPUT);

    digitalWrite(STATUS_LED, HIGH);
    digitalWrite(MOTOR_EN, HIGH);
}

void loop()
{
    for (int value = 0; value < 255; value++) {
        set_m1(value);
        set_m2(value);
        set_m3(value);
        set_m4(value);
        delay(50);
        report_encoders();
    }

    for (int value = 255; value >= -255; value--) {
        set_m1(value);
        set_m2(value);
        set_m3(value);
        set_m4(value);
        delay(50);
        report_encoders();
    }

    for (int value = -255; value < 0; value++) {
        set_m1(value);
        set_m2(value);
        set_m3(value);
        set_m4(value);
        delay(50);
        report_encoders();
    }
}

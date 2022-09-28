#include <Arduino.h>
#include <Encoder.h>


const int MOTOR1_ENCA = 3;
const int MOTOR1_ENCB = 4;
const int MOTOR2_ENCA = 5;
const int MOTOR2_ENCB = 6;

Encoder enc1(MOTOR1_ENCA, MOTOR1_ENCB);
Encoder enc2(MOTOR2_ENCA, MOTOR2_ENCB);

void setup()
{
    Serial.begin(9600);
    
    enc1.write(0);
    enc2.write(0);
}

void loop()
{
    long enc1_value = enc1.read();
    long enc2_value = enc2.read();

    Serial.print(enc1_value);
    Serial.print('\t');
    Serial.print(enc2_value);
    Serial.print('\n');
    delay(100);
}

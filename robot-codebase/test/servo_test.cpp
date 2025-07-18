//
// Created by bram on 18/07/25.
//
#include "hardware/CustomServo.h"

constexpr int servoPin = 21;
constexpr int pwmChannel = 1;
CustomServo testServo(servoPin, pwmChannel);
void setup()
{
    //do nothing
}

void loop()
{
    testServo.setAngle(180);
    delay(2000);
    testServo.setAngle(0);
    delay(2000);
}

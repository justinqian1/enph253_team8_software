//
// Created by bram on 16/07/25.
//

#ifndef SERVO_H
#define SERVO_H
#include "interfaces/servo_interface.h"
#include "ESP32Servo.h"

class CustomServo : public servo_interface
{
    public:
    explicit CustomServo(int pin);
    explicit CustomServo(int pin, int position);
    int getPin();
    int getPosition();
    void setAngle(int angle) override;
    void setAngle(int angle, int time) override;

    private:
    int servoPin;
    int servoPosition;
    Servo servo;
};
#endif //SERVO_H

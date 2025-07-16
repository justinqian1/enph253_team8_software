//
// Created by bram on 16/07/25.
//

#ifndef MOTOR_H
#define MOTOR_H
#include "interfaces/motor_interface.h"

class Motor : public motor_interface
{
    public:
    explicit Motor(int pwmPin, int directionPin);
    void driveMotor(int speed, int direction) override;
    void stopMotor();
    void driveForward(int speed) override;
    void driveReverse(int speed) override;
    int getMotorPWMPin();
    int getMotorDirectionPin();

    private:
    int motorPWMPin;
    int motorDirectionPin;
};
#endif //MOTOR_H

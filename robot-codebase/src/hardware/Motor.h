//
// Created by bram on 16/07/25.
//

#ifndef MOTOR_H
#define MOTOR_H
#include "interfaces/motor_interface.h"
#include "Arduino.h"
#include "driver/ledc.h"

class Motor : public motor_interface
{
    public:
    explicit Motor(int pwmCh);
    void attachPins(int pwmPin, int dirPin);
    void driveMotor(int speed, int direction) override;
    void stopMotor();
    void driveForward(int speed) override;
    void driveReverse(int speed) override;
    void drivePID(int avgSpeed);
    int getMotorPWMPin();
    int getMotorDirectionPin();

    private:
    int motorPWMPin = 0;
    int motorDirectionPin = 0;
    int currentDirection = 0;
    int pwmChannel;
};
#endif //MOTOR_H

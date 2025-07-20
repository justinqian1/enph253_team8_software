//
// Created by bram on 16/07/25.
//

#include "Motor.h"

Motor::Motor(int pwmCh) : pwmChannel(pwmCh)
{
    ledcSetup(this->pwmChannel, 250,12);
}

void Motor::attachPins(int pwmPin, int dirPin)
{
    this->motorPWMPin = pwmPin;
    this->motorDirectionPin = dirPin;
    ledcAttachPin(this->motorPWMPin, pwmChannel);
    digitalWrite(this->motorDirectionPin, LOW);
}

void Motor::driveMotor(int speed, int direction)
{
    if (direction == this->currentDirection)
    {
        ledcWrite(this->motorPWMPin, speed);
    } else
    {
        digitalWrite(this->motorDirectionPin, direction);
        ledcWrite(this->motorPWMPin, speed);
        this->currentDirection = direction;
    }
}

void Motor::stopMotor()
{
    ledcWrite(this->motorPWMPin, 0);
}

void Motor::driveForward(int speed)
{
    driveMotor(speed, HIGH);
}

void Motor::driveReverse(int speed)
{
    driveMotor(speed, LOW);
}

int Motor::getMotorPWMPin()
{
    return this->motorPWMPin;
}

int Motor::getMotorDirectionPin()
{
    return this->motorDirectionPin;
}
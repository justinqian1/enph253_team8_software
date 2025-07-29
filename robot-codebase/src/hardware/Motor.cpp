//
// Created by bram on 16/07/25.
//

#include "Motor.h"
#include "constants.h"

Motor::Motor(int pwmCh, int pwmPin, int directionPin) : pwmChannel(pwmCh), motorPWMPin(pwmPin), motorDirectionPin(directionPin)
{
    ledcSetup(pwmCh, pwmFreq, 12);
    ledcAttachPin(this->motorPWMPin, this->pwmChannel);
    pinMode(this->motorDirectionPin, OUTPUT);
}



void Motor::driveMotor(int speed, int direction)
{
    if (direction == this->currentDirection)
    {
        ledcWrite(this->pwmChannel, speed);
        Serial.println("motor driving");
    } else
    {
        stopMotor();
        Serial.println("motor direction changing");
        vTaskDelay(5 / portTICK_PERIOD_MS);
        digitalWrite(this->motorDirectionPin, direction);
        ledcWrite(this->pwmChannel, speed);
        this->currentDirection = direction;
    }
}

void Motor::stopMotor()
{
    ledcWrite(this->pwmChannel, 0);
}

void Motor::driveForward(int speed)
{
    if (this->currentDirection == HIGH)
    {
        ledcWrite(this->pwmChannel, speed);
    } else
    {
        stopMotor();
        vTaskDelay(5 / portTICK_PERIOD_MS);
        digitalWrite(this->motorDirectionPin, HIGH);
        ledcWrite(this->pwmChannel, speed);
        this->currentDirection = HIGH;
    }
}

void Motor::driveReverse(int speed)
{
    if (this->currentDirection == LOW)
    {
        ledcWrite(this->pwmChannel, speed);
    } else
    {
        stopMotor();
        vTaskDelay(5 / portTICK_PERIOD_MS);
        digitalWrite(this->motorDirectionPin, LOW);
        ledcWrite(this->pwmChannel, speed);
        this->currentDirection = LOW;
    }
}

int Motor::getMotorPWMPin()
{
    return this->motorPWMPin;
}

int Motor::getMotorDirectionPin()
{
    return this->motorDirectionPin;
}
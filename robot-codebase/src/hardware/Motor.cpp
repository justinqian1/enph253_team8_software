//
// Created by bram on 16/07/25.
//

#include "Motor.h"
#include "constants.h"

Motor::Motor(int pwmCh) : pwmChannel(pwmCh)
{
    ledcSetup(pwmCh, pwmFreq, 12);
    Serial.print("Motor created with channel");
    Serial.println(pwmCh);
}

void Motor::attachPins(int pwmCh, int pwmPin, int dirPin)
{
    this->motorPWMPin = pwmPin;
    this->motorDirectionPin = dirPin;
    ledcSetup(pwmCh, pwmFreq, 12);
    ledcAttachPin(this->motorPWMPin, pwmChannel);
    pinMode(dirPin, OUTPUT);
    digitalWrite(this->motorDirectionPin, HIGH);
}

void Motor::driveMotor(int speed, int direction)
{
    if (direction == this->currentDirection)
    {
        ledcWrite(this->motorPWMPin, speed);
        Serial.println("motor driving");
    } else
    {
        stopMotor();
        Serial.println("motor direction changing");
        vTaskDelay(5 / portTICK_PERIOD_MS);
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
    if (this->currentDirection == HIGH)
    {
        ledcWrite(this->motorPWMPin, speed);
    } else
    {
        stopMotor();
        vTaskDelay(5 / portTICK_PERIOD_MS);
        digitalWrite(this->motorDirectionPin, HIGH);
        ledcWrite(this->motorPWMPin, speed);
        this->currentDirection = HIGH;
    }
}

void Motor::driveReverse(int speed)
{
    if (this->currentDirection == LOW)
    {
        ledcWrite(this->motorPWMPin, speed);
    } else
    {
        stopMotor();
        vTaskDelay(5 / portTICK_PERIOD_MS);
        digitalWrite(this->motorDirectionPin, LOW);
        ledcWrite(this->motorPWMPin, speed);
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
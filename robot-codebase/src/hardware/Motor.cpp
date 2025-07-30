//
// Created by bram on 16/07/25.
//

#include "Motor.h"
#include "constants.h"

// comment this out to remove debugging code
#define MOTOR_DEBUG
#define FORWARD 1
#define BACKWARD 0

Motor::Motor(int pwmChFwd, int pwmFwdPin, int pwmChBkwd, int pwmBkwdPin) : forwardPWMChannel(pwmChFwd), forwardPWMPin(pwmFwdPin), backwardPWMChannel(pwmChBkwd), backwardPWMPin(pwmBkwdPin)
{
    ledcSetup(forwardPWMChannel, pwmFreq, 12);
    ledcSetup(backwardPWMChannel, pwmFreq, 12);
    ledcAttachPin(this->forwardPWMPin, forwardPWMChannel);
    ledcAttachPin(this->backwardPWMPin, backwardPWMChannel);
    #ifdef MOTOR_DEBUG
    Serial.println("Motor created!");
    #endif 
}

void Motor::driveMotor(int speed, int direction)
{
    switch (direction) {
        case FORWARD:
            if (this->currentDirection == FORWARD) {
                ledcWrite(this->forwardPWMChannel, speed);
            } else {
                ledcWrite(this->backwardPWMChannel, 0);
                //vTaskDelay(deadTime / portTICK_PERIOD_MS);
                ledcWrite(this->forwardPWMChannel,  speed);
                this->currentDirection = direction;
            }
                #ifdef MOTOR_DEBUG
                Serial.println("Driving forward!!");
                #endif
            break;
        case BACKWARD:
            if (this->currentDirection == BACKWARD) {
                ledcWrite(this->backwardPWMChannel, speed);
            } else {
                ledcWrite(this->forwardPWMChannel, 0);
                //vTaskDelay(deadTime / portTICK_PERIOD_MS);
                ledcWrite(this->backwardPWMChannel,  speed);
                this->currentDirection = direction;
            }
                #ifdef MOTOR_DEBUG
                Serial.println("Driving backward!!");
                #endif
            break;
        default:
            // do nothing
            break;
    }
}

void Motor::stopMotor()
{
    ledcWrite(this->forwardPWMChannel, 0);
    ledcWrite(this->backwardPWMChannel, 0);
}

void Motor::driveForward(int speed)
{
   driveMotor(speed, FORWARD);
}

void Motor::driveReverse(int speed)
{
    driveMotor(speed, BACKWARD);
}

int Motor::getForwardPWMPin()
{
    return this->forwardPWMPin;
}

int Motor::getBackwardPWMPin()
{
    return this->backwardPWMPin;
}
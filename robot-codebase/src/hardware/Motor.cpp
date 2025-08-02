//
// Created by bram on 16/07/25.
//

#include "Motor.h"
#include "constants.h"

// comment this out to remove debugging code
//#define MOTOR_DEBUG
#define FORWARD 1
#define BACKWARD 0

int created = 0;
Motor::Motor(int pwmChFwd, int pwmFwdPin, int pwmChBkwd, int pwmBkwdPin) : _forwardPWMChannel(pwmChFwd), _forwardPWMPin(pwmFwdPin), _backwardPWMChannel(pwmChBkwd), _backwardPWMPin(pwmBkwdPin)
{
    ledcSetup(_forwardPWMChannel, pwmFreq, 12);
    ledcSetup(_backwardPWMChannel, pwmFreq, 12);
    ledcAttachPin(this->_forwardPWMPin, _forwardPWMChannel);
    ledcAttachPin(this->_backwardPWMPin, _backwardPWMChannel);
    created = 1;
    #ifdef MOTOR_DEBUG
    Serial.println("Motor created!");
    #endif 
}

void Motor::driveMotor(int speed, int direction)
{
    switch (direction) {
        case FORWARD:
            if (this->currentDirection == FORWARD) {
                ledcWrite(this->_forwardPWMChannel, speed);
            } else {
                ledcWrite(this->_backwardPWMChannel, 0);
                //vTaskDelay(deadTime / portTICK_PERIOD_MS);
                ledcWrite(this->_forwardPWMChannel,  speed);
                this->currentDirection = direction;
            }
                #ifdef MOTOR_DEBUG
                Serial.println("Driving forward!!");
                #endif
            break;
        case BACKWARD:
            if (this->currentDirection == BACKWARD) {
                ledcWrite(this->_backwardPWMChannel, speed);
            } else {
                ledcWrite(this->_forwardPWMChannel, 0);
                //vTaskDelay(deadTime / portTICK_PERIOD_MS);
                ledcWrite(this->_backwardPWMChannel,  speed);
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
    //Serial.print("Motor:");
    //Serial.println(created);
}

void Motor::stopMotor()
{
    ledcWrite(this->_forwardPWMChannel, 0);
    ledcWrite(this->_backwardPWMChannel, 0);
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
    return this->_forwardPWMPin;
}

int Motor::getBackwardPWMPin()
{
    return this->_backwardPWMPin;
}
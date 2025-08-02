//
// Created by bram on 16/07/25.
//
#include <Arduino.h>
#include "CustomServo.h"


CustomServo::CustomServo(int pin, int channel) : _servoPin(pin), _pwmChannel(channel), _servoPosition(0), _periodHertz(50),
                                                 _minPulse(1000), _maxPulse(2000), _rotationMultiplier(1.0)
{
    ledcSetup(this->_pwmChannel, 50, 16);
    ledcAttachPin(this->_servoPin, this->_pwmChannel);
}

CustomServo::CustomServo(int pin, int channel, int position) : _servoPin(pin), _pwmChannel(channel),
                                                               _servoPosition(position), _periodHertz(50), _minPulse(1000),
                                                               _maxPulse(2000), _rotationMultiplier(1.0)
{
    ledcSetup(this->_pwmChannel, 50, 16);
    ledcAttachPin(this->_servoPin, this->_pwmChannel);
}

CustomServo::CustomServo(int pin, int channel, int position, int hertz) : _servoPin(pin), _pwmChannel(channel),
                                                                          _servoPosition(position), _periodHertz(hertz),
                                                                          _minPulse(1000), _maxPulse(2000), _rotationMultiplier(1.0)
{
    ledcSetup(this->_pwmChannel, this->_periodHertz, 16);
    ledcAttachPin(this->_servoPin, this->_pwmChannel);
}

CustomServo::CustomServo(int pin, int channel, int position, int hertz, unsigned long min, unsigned long max) :
    _servoPin(pin), _pwmChannel(channel), _servoPosition(position), _periodHertz(hertz), _minPulse(min), _maxPulse(max), 
    _rotationMultiplier(1.0)
{
    ledcSetup(this->_pwmChannel, this->_periodHertz, 16);
    ledcAttachPin(this->_servoPin, this->_pwmChannel);
}

CustomServo::CustomServo(int pin, int channel, int position, int hertz, unsigned long min, unsigned long max, double multiplier) :
    _servoPin(pin), _pwmChannel(channel), _servoPosition(position), _periodHertz(hertz), _minPulse(min), _maxPulse(max), 
    _rotationMultiplier(multiplier)
{
    ledcSetup(this->_pwmChannel, this->_periodHertz, 16);
    ledcAttachPin(this->_servoPin, this->_pwmChannel);
}

int CustomServo::getPosition() { return this->_servoPosition; }

int CustomServo::getPin() { return this->_servoPin; }

void CustomServo::rotateTo(int position)
{
    position = constrain(position, 0, 180*this->_rotationMultiplier);
    int posAsLedcValue = posToLedcWrite(position);
    //Serial2Pi.print("writing ledc position");
    //Serial2Pi.println(posAsLedcValue);
    int pulse_us = pulseLength(posAsLedcValue);
    uint32_t duty = dutyCycle(pulse_us) * _maxDuty;
    Serial.print("Servo Pin: ");
    Serial.println(this->_servoPin);
    Serial.print("Duty: ");
    Serial.println(duty);
    ledcWrite(this->_pwmChannel, duty);
    this->_servoPosition = position;
}

void CustomServo::rotateTo(int position, int time)
{
    int numTicks = abs(this->_servoPosition - position);
    int tickTime = time / numTicks;
    if (this->_servoPosition < position)
    {
        while (this->_servoPosition < position)
        {
            rotateTo(++this->_servoPosition);
            vTaskDelay(tickTime / portTICK_PERIOD_MS);
        }
    }
    else if (this->_servoPosition > position)
    {
        while (this->_servoPosition > position)
        {
            rotateTo(--this->_servoPosition);
            vTaskDelay(tickTime / portTICK_PERIOD_MS);
        }
    }
}   

void CustomServo::write(int position){ 
    rotateTo(position);
}

void CustomServo::rotateBy(int degrees) {
    //Serial2Pi.print("rotating to position");
    //Serial2Pi.println(_servoPosition + degrees);
    rotateTo(this->_servoPosition + degrees);
}

//PRIVATE FUNCTIONS

int CustomServo::pulseLength(int pos)
{
    Serial.print("Map: ");
    Serial.println(map(pos,0,180,_minPulse,_maxPulse));
    return map(pos, 0, 180, this->_minPulse, this->_maxPulse);
}

double CustomServo::dutyCycle(int length)
{
    Serial.print("Duty Function: ");
    Serial.println(1e6 / static_cast<double>(_periodHertz));
    double period = 1e6 / static_cast<double>(this->_periodHertz);
    return (length) / (double)period;
}

int CustomServo::posToLedcWrite(int pos) {
    Serial.print("Multiplier: ");
    Serial.println((int)round((double)pos / _rotationMultiplier));
    Serial.print("Mult Value : ");
    Serial.println(_rotationMultiplier);
    return (int)round((double)pos / this->_rotationMultiplier);
}

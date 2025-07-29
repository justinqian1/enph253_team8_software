//
// Created by bram on 16/07/25.
//
#include <Arduino.h>
#include "CustomServo.h"


CustomServo::CustomServo(int pin, int channel) : servoPin(pin), pwmChannel(channel), servoPosition(0), periodHertz(50),
                                                 minPulse(1000), maxPulse(2000), rotationMultiplier(1.0)
{
    ledcSetup(this->pwmChannel, 50, 16);
    ledcAttachPin(this->servoPin, this->pwmChannel);
}

CustomServo::CustomServo(int pin, int channel, int position) : servoPin(pin), pwmChannel(channel),
                                                               servoPosition(position), periodHertz(50), minPulse(1000),
                                                               maxPulse(2000), rotationMultiplier(1.0)
{
    ledcSetup(this->pwmChannel, 50, 16);
    ledcAttachPin(this->servoPin, this->pwmChannel);
}

CustomServo::CustomServo(int pin, int channel, int position, int hertz) : servoPin(pin), pwmChannel(channel),
                                                                          servoPosition(position), periodHertz(hertz),
                                                                          minPulse(1000), maxPulse(2000), rotationMultiplier(1.0)
{
    ledcSetup(this->pwmChannel, this->periodHertz, 16);
    ledcAttachPin(this->servoPin, this->pwmChannel);
}

CustomServo::CustomServo(int pin, int channel, int position, int hertz, unsigned long min, unsigned long max) :
    servoPin(pin), pwmChannel(channel), servoPosition(position), periodHertz(hertz), minPulse(min), maxPulse(max), 
    rotationMultiplier(1.0)
{
    ledcSetup(this->pwmChannel, this->periodHertz, 16);
    ledcAttachPin(this->servoPin, this->pwmChannel);
}

CustomServo::CustomServo(int pin, int channel, int position, int hertz, unsigned long min, unsigned long max, double multiplier) :
    servoPin(pin), pwmChannel(channel), servoPosition(position), periodHertz(hertz), minPulse(min), maxPulse(max), 
    rotationMultiplier(multiplier)
{
    ledcSetup(this->pwmChannel, this->periodHertz, 16);
    ledcAttachPin(this->servoPin, this->pwmChannel);
}

int CustomServo::getPosition() { return this->servoPosition; }

int CustomServo::getPin() { return this->servoPin; }

void CustomServo::rotateTo(int position)
{
    position = constrain(position, 0, 180*this->rotationMultiplier);
    int posAsLedcValue = posToLedcWrite(position);
    Serial.print("writing ledc position");
    Serial.println(posAsLedcValue);
    int pulse_us = pulseLength(posAsLedcValue);
    uint32_t duty = dutyCycle(pulse_us) * maxDuty;
    ledcWrite(this->pwmChannel, duty);
    this->servoPosition = position;
}
/*
multiplier=2 for simplicity
start at 180, want to go to 90
getPosition() returns 180
write pwm of 45 (90 / multiplier)
    - pwm is the only thing that doesnt line up to absolute position
servo position however returns duty*multiplier
*/

void CustomServo::rotateTo(int position, int time)
{
    int numTicks = abs(this->servoPosition - position);
    int tickTime = time / numTicks;
    if (this->servoPosition < position)
    {
        while (this->servoPosition < position)
        {
            rotateTo(++this->servoPosition);
            vTaskDelay(tickTime / portTICK_PERIOD_MS);
        }
    }
    else if (this->servoPosition > position)
    {
        while (this->servoPosition > position)
        {
            rotateTo(--this->servoPosition);
            vTaskDelay(tickTime / portTICK_PERIOD_MS);
        }
    }
}

void CustomServo::write(int position){ 
    rotateTo(position);
}

void CustomServo::rotateBy(int degrees) {
    Serial.print("rotating to position");
    Serial.println(servoPosition + degrees);
    rotateTo(this->servoPosition + degrees);
}

//PRIVATE FUNCTIONS

int CustomServo::pulseLength(int pos)
{
    return map(pos, 0, 180, this->minPulse, this->maxPulse);
}

double CustomServo::dutyCycle(int length)
{
    double period = 1e6 / static_cast<double>(this->periodHertz);
    return (length) / (double)period;
}

int CustomServo::posToLedcWrite(int pos) {
    return (int)round((double)pos / this->rotationMultiplier);
}

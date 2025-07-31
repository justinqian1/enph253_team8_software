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
    //Serial2Pi.print("writing ledc position");
    //Serial2Pi.println(posAsLedcValue);
    int pulse_us = pulseLength(posAsLedcValue);
    uint32_t duty = dutyCycle(pulse_us) * maxDuty;
    Serial.print("Servo Pin: ");
    Serial.println(this->servoPin);
    Serial.print("Duty: ");
    Serial.println(duty);
    ledcWrite(this->pwmChannel, duty);
    this->servoPosition = position;
}

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
    //Serial2Pi.print("rotating to position");
    //Serial2Pi.println(servoPosition + degrees);
    rotateTo(this->servoPosition + degrees);
}

//PRIVATE FUNCTIONS

int CustomServo::pulseLength(int pos)
{
    Serial.print("Map: ");
    Serial.println(map(pos,0,180,minPulse,maxPulse));
    return map(pos, 0, 180, this->minPulse, this->maxPulse);
}

double CustomServo::dutyCycle(int length)
{
    Serial.print("Duty Function: ");
    Serial.println(1e6 / static_cast<double>(periodHertz));
    double period = 1e6 / static_cast<double>(this->periodHertz);
    return (length) / (double)period;
}

int CustomServo::posToLedcWrite(int pos) {
    Serial.print("Multiplier: ");
    Serial.println((int)round((double)pos / rotationMultiplier));
    Serial.print("Mult Value : ");
    Serial.println(rotationMultiplier);
    return (int)round((double)pos / this->rotationMultiplier);
}

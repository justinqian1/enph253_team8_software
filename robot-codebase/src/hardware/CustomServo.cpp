//
// Created by bram on 16/07/25.
//
#include <Arduino.h>
#include "CustomServo.h"


/**
 * creates a CustomServo object with a default position set to 0
 * @param pin the pin to which this servo is assigned
 * @param channel the PWM channel assigned to this servo
 */
CustomServo::CustomServo(int pin, int channel) : servoPin(pin), pwmChannel(channel), servoPosition(0), periodHertz(50), minPulse(1000), maxPulse(2000) 
{
    ledcSetup(this->pwmChannel, 50, 16);
    ledcAttachPin(this->servoPin, this->pwmChannel);
}
/**
 * creates a CustomServo object with a starting position
 * @param pin the pin to which this servo is assigned
 * @param channel the  pwm channel assigned to this servo
 * @param position the initial position of this servo
 */
CustomServo::CustomServo(int pin, int channel, int position) : servoPin(pin), pwmChannel(channel), servoPosition(position), periodHertz(50), minPulse(1000), maxPulse(2000) {
    ledcSetup(this->pwmChannel, 50, 16);
    ledcAttachPin(this->servoPin, this->pwmChannel);
}

/**
 * creates a CustomServo object with a variety of parameters
 * @param pin the pin to which this servo is assigned
 * @param channel the PWM channel assigned to this servo
 * @param position the initial position of this servo
 * @param hertz the period used for this servo
 */
CustomServo::CustomServo(int pin, int channel, int position, int hertz) : servoPin(pin), pwmChannel(channel), servoPosition(position),  periodHertz(hertz), minPulse(1000), maxPulse(2000) {
    ledcSetup(this->pwmChannel, this->periodHertz, 16);
    ledcAttachPin(this->servoPin, this->pwmChannel);
}

/**
 * creates a CustomServo object with a wider variety of parameters
 * @param pin the pin to which this servo is assigned
 * @param channel the PWM channel assigned to this servo
 * @param position the initial position of this servo
 * @param hertz the period used for this servo
 * @param min the minimum pulse length in microseconds
 * @param max the maximum pulse length in microseconds
 */
CustomServo::CustomServo(int pin, int channel, int position, int hertz, unsigned long min,  unsigned long max) : servoPin(pin), pwmChannel(channel), servoPosition(position), periodHertz(hertz), minPulse(min), maxPulse(max) {
    ledcSetup(this->pwmChannel, this->periodHertz, 16);
    ledcAttachPin(this->servoPin, this->pwmChannel);
}
/**
 * returns the current position of the servo
 * @return the current position of the servo
 */
int CustomServo::getPosition() { return this->servoPosition; }

/**
 * returns the pin assignment of this servo
 * @return the pin assigned to this servo
 */
int CustomServo::getPin() { return this->servoPin; }

/**
 * sets the position of a servo to a specified angle
 * @param position the position (in degrees) to set the servo to
 */
void CustomServo::setAngle(int position)
{
    position = constrain(position, 0, 180);
    int pulse_us = pulseLength(position);
    uint32_t duty = dutyCycle(pulse_us)*maxDuty;
    ledcWrite(this->pwmChannel, duty);
    this->servoPosition = position;
}

/**
 * sets the position of a servo to a specified angle over a time frame (used to slow down servo rotation)
 * @param position the position (in degrees) to set the servo to
 * @param time the time (in milliseconds) to turn the servo
 */
void CustomServo::setAngle(int position, int time)
{
    int numTicks = abs(this->servoPosition - position);
    int tickTime = time / numTicks;
    if (this->servoPosition < position)
    {
        while (this->servoPosition < position)
        {
            setAngle(++this->servoPosition);
            vTaskDelay(tickTime / portTICK_PERIOD_MS);
        }
    } else if (this->servoPosition > position)
    {
        while (this->servoPosition > position)
        {
           setAngle(--this->servoPosition);
            vTaskDelay(tickTime / portTICK_PERIOD_MS);
        }
    }
}

//PRIVATE FUNCTIONS

/**
 * Private function that calculate the length of a pulse required to get to a certain position, in microseconds
 * @param pos the position to calculate the pulse for
 * @return the length of the pulse, in microseconds
 */
int CustomServo::pulseLength(int pos) {
    return map(pos, 0, 180, this->minPulse, this->maxPulse);
}

/**
 * Private function that calculates the duty cycle required to make the PWM signal produce a certain pulse length
 * @param length the length of the pulse to produce
 * @return a double percentage that represents the % duty cycle
 */
double CustomServo::dutyCycle(int length) {
    double period = 1e6 / static_cast<double>(this->periodHertz);
    return (length) / (double)period;
}


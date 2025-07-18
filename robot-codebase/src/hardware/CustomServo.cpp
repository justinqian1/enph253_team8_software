//
// Created by bram on 16/07/25.
//
#include <Arduino.h>
#include "CustomServo.h"
/**
 * creates a CustomServo object with a default position set to 0
 * @param pin the pin to which this servo is assigned
 */
CustomServo::CustomServo(int pin, int channel) : servoPin(pin), pwmChannel(channel), servoPosition(0), periodHertz(50), minPulse(1000), maxPulse(2000) 
{
    ledcSetup(pwmChannel, 50, 2000);
    ledcAttachPin(servoPin, pwmChannel);
}
/**
 * creates a CustomServo object with a starting position
 * @param pin the pin to which this servo is assigned
 * @param position the initial position of this servo
 */
CustomServo::CustomServo(int pin, int channel, int position) : servoPin(pin), pwmChannel(channel), servoPosition(position), periodHertz(50), minPulse(1000), maxPulse(2000) {
    ledcSetup(pwmChannel, 50, 2000);
    ledcAttachPin(servoPin, pwmChannel);
}

CustomServo::CustomServo(int pin, int channel, int position, int hertz) : servoPin(pin), pwmChannel(channel), servoPosition(position),  periodHertz(hertz), minPulse(1000), maxPulse(2000) {
    ledcSetup(pwmChannel, periodHertz, 2000);
    ledcAttachPin(servoPin, pwmChannel);
}

CustomServo::CustomServo(int pin, int channel, int position, int hertz, unsigned long min,  unsigned long max) : servoPin(pin), pwmChannel(channel), servoPosition(position), periodHertz(hertz), minPulse(min), maxPulse(max) {
    ledcSetup(pwmChannel, periodHertz, 2000);
    ledcAttachPin(servoPin, pwmChannel);
}
/**
 * returns the current position of the servo
 * @return the current position of the servo
 */
int CustomServo::getPosition() { return servoPosition; }

/**
 * returns the pin assignment of this servo
 * @return the pin assigned to this servo
 */
int CustomServo::getPin() { return servoPin; }

/**
 * sets the position of a servo to a specified angle
 * @param position the position (in degrees) to set the servo to
 */
void CustomServo::setAngle(int position)
{
    
    servoPosition = position;
}

/**
 * sets the position of a servo to a specified angle over a time frame (used to slow down servo rotation)
 * @param position the position (in degrees) to set the servo to
 * @param time the time (in milliseconds) to turn the servo
 */
void CustomServo::setAngle(int position, int time)
{
    int numTicks = abs(servoPosition - position);
    int tickTime = time / numTicks;
    if (servoPosition < position)
    {
        while (servoPosition < position)
        {
            servo.write(++servoPosition);
            vTaskDelay(tickTime / portTICK_PERIOD_MS);
        }
    } else if (servoPosition > position)
    {
        while (servoPosition > position)
        {
            servo.write(--servoPosition);
            vTaskDelay(tickTime / portTICK_PERIOD_MS);
        }
    }

    //PRIVATE FUNCTIONS
    /**
     * 
     */
    int pulseLength(int pos) {
        return (int) (maxPulse - minPulse) * pos / 180;
    }

    /**
     * 
     */
    int dutyCycle(int pulseLength) {
        double period = (1 / (double) periodHertz) * 10^6;
        return pulseLength / period;
    }
}

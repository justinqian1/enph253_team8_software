//
// Created by bram on 16/07/25.
//

#include "CustomServo.h"

/**
 * creates a CustomServo object with a default position set to 0
 * @param pin the pin to which this servo is assigned
 */
CustomServo::CustomServo(int pin) : servoPin(pin), servoPosition(0)
{
    servo.setPeriodHertz(50);
    servo.attach(servoPin);
    servo.write(0);
}

/**
 * creates a CustomServo object with a starting position
 * @param pin the pin to which this servo is assigned
 * @param position the initial position of this servo
 */
CustomServo::CustomServo(int pin, int position) : servoPin(pin), servoPosition(position)
{
    servo.setPeriodHertz(50);
    servo.attach(servoPin);
    servo.write(servoPosition);
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
    servo.write(position);
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
}

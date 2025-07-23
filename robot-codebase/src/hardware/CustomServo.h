//
// Created by bram on 16/07/25.
//

#ifndef SERVO_H
#define SERVO_H
#include "interfaces/servo_interface.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include <Arduino.h>

class CustomServo : public servo_interface {
public:
    /**
     * creates a CustomServo object with a default position set to 0
     * @param pin the pin to which this servo is assigned
     * @param channel the PWM channel assigned to this servo
     */
    CustomServo(int pin, int channel);

    /**
     * creates a CustomServo object with a starting position
     * @param pin the pin to which this servo is assigned
     * @param channel the  pwm channel assigned to this servo
     * @param position the initial position of this servo
     */
    CustomServo(int pin, int channel, int position);

    /**
     * creates a CustomServo object with a variety of parameters
     * @param pin the pin to which this servo is assigned
     * @param channel the PWM channel assigned to this servo
     * @param position the initial position of this servo
     * @param hertz the period used for this servo
     */
    CustomServo(int pin, int channel, int position, int period);

    /**
     * creates a CustomServo object with a wider variety of parameters
     * @param pin the pin to which this servo is assigned
     * @param channel the PWM channel assigned to this servo
     * @param position the initial position of this servo
     * @param hertz the period used for this servo
     * @param min the minimum pulse length in microseconds
     * @param max the maximum pulse length in microseconds
     */
    CustomServo(int pin, int channel, int position, int period, unsigned long min, unsigned long max);

    /**
     * returns the pin assignment of this servo
     * @return the pin assigned to this servo
     */
    int getPin();

    /**
     * returns the current position of the servo
     * @return the current position of the servo
     */
    int getPosition();

    /**
     * sets the position of a servo to a specified angle
     * @param angle the position (in degrees) to set the servo to
     */
    void rotateTo(int angle) override;

    /**
     * sets the position of a servo to a specified angle over a time frame (used to slow down servo rotation)
     * @param position the position (in degrees) to set the servo to
     * @param time the time (in milliseconds) to turn the servo
     */
    void rotateTo(int angle, int time) override;

    /**
     * alias for Servo.write() function
     */
    void write(int angle);
    
    /**
     * rotates the servo a certain number of degrees, with positive being clockwise
     * @param degrees the number of degrees to rotate
     */
    void rotateBy(int degrees);

protected:
    int servoPin;
    int servoPosition;
    int periodHertz;
    int pwmChannel;
    static constexpr int maxDuty = (1 << 16) - 1;
    unsigned long minPulse;
    unsigned long maxPulse;
    /**
     * Private function that calculate the length of a pulse required to get to a certain position, in microseconds
     * @param pos the position to calculate the pulse for
     * @return the length of the pulse, in microseconds
     */
    int pulseLength(int pos);

    /**
     * Private function that calculates the duty cycle required to make the PWM signal produce a certain pulse length
     * @param length the length of the pulse to produce
     * @return a double percentage that represents the % duty cycle
     */
    double dutyCycle(int length);
};
#endif //SERVO_H

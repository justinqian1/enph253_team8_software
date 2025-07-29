//
// Created by bram on 16/07/25.
//

#ifndef MOTOR_H
#define MOTOR_H
#include "interfaces/motor_interface.h"
#include "Arduino.h"
#include "driver/ledc.h"

class Motor : public motor_interface
{
    public:
    /**
     * Create a motor object with two PWM channels for forward and backward, attached to two pins
     * @param pwmChFwd the PWM channel to run forward movement
     * @param pwmFwdPin the pin assigned to the forward PWM channel
     * @param pwmChBkwd the PWM channel to run backward movement
     * @param pwmBkwdPin tne pin assigned to the backward PWM channel
     */
    Motor(int pwmChFwd, int pwmFwdPin, int pwmChBkwd, int pwmBkwdPin);
    /**
     * drives the motor with a specified speed and direction, with built in shoot-through protection
     * @param speed the speed at which to drive the motor, from 0 to 4095
     * @param direction the direction in which to drive the motor, 1 for forward, 0 for backward
     */
    void driveMotor(int speed, int direction) override;

    /**
     * Stops the motor by assigning it a PWM value of 0 **does not change the motor direction
     */
    void stopMotor();

    /**
     * Drives the motor in the forward direction at a specified speed (direction = 1)
     * @param speed the speed at which to drive the motor
     */
    void driveForward(int speed) override;

    /**
     * Drives the motor in the backward direction at a specified speed (direction = 0)
     * @param speed
     */
    void driveReverse(int speed) override;

    /**
     * returns the PWM channel pin
     * @return the pin for the PWM channel
     */
    int getForwardPWMPin();

    /**
     * returns the direction pin
     * @return the pin for the direction
     */
    int getBackwardPWMPin();

    int currentDirection = 0;

    private:
    int forwardPWMPin;
    int backwardPWMPin;
    int forwardPWMChannel;
    int backwardPWMChannel;
    constexpr int deadTime = 10; // ms
};
#endif //MOTOR_H

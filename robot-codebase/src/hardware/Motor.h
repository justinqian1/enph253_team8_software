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
     * Creates a motor object assigned to a specific PWM channel
     * @param pwmCh
     */
    Motor(int pwmCh);

    /**
     * Assigns PWM and direction pins to a motor object
     * @param pwmPin the pin assigned to the motor PWM
     * @param dirPin the pin assigned to the motor direction
     */
    void attachPins(int pwmPin, int dirPin);

    /**
     * drives the motor with a specified speed and direction
     * @param speed the speed at which to drive the motor, from 0 to 4095
     * @param direction the direction in which to drive the motor, 0 for right, 1 for left
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
    int getMotorPWMPin();

    /**
     * returns the direction pin
     * @return the pin for the direction
     */
    int getMotorDirectionPin();

    private:
    int motorPWMPin = 0;
    int motorDirectionPin = 0;
    int currentDirection = 0;
    int pwmChannel;
};
#endif //MOTOR_H

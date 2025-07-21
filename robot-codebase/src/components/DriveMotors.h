//
// Created by bram on 16/07/25.
//

#ifndef DRIVEMOTORS_H
#define DRIVEMOTORS_H

#include "hardware/CustomServo.h"
#include "hardware/Motor.h"

class DriveMotors
{
public:
    /**
     * Creates a DriveMotor object with two motors to drive the robot
     * @param leftMotor the left motor
     * @param rightMotor the right motor
     */
    DriveMotors(const Motor& leftMotor, const Motor& rightMotor);

    /**
     * Drives the robot with PID control
     * @param speed the average speed to drive the robot at
     */
    void drivePID(int speed);

    /**
     * Drives the robot with PID control, with variable parameters
     * @param speed the average speed to drive the robot at
     * @param proportional the proportional term for the PID
     * @param derivative the derivative term for the PID
     * @param integral the integral term for the PID
     */
    void drivePID(int speed, double proportional, double derivative, double integral);

    /**
     * Stops the robot by stopping both motors
     */
    void stop();

    /**
     * Drives the robot without PID control
     * @param speed the speed at which to drive the robot
     * @param direction the direction to drive straight
     */
    void driveStraight(int speed, int direction);

    /**
     * drives the left motor
     * @param speed the speed at which to drive the left motor
     * @param direction the direction to drive the left motor
     */
    void driveLeftMotor(int speed, int direction);

    /**
     * drives the right motor
     * @param speed the speed at which to drive the right motor
     * @param direction the direction to drive the left motor
     */
    void driveRightMotor(int speed, int direction);

private:
    int leftMotorDirection = 1;
    int rightMotorDirection = 1;
    int currentSpeed = 0;
    Motor leftMotor;
    Motor rightMotor;
};
#endif //DRIVEMOTORS_H

//
// Created by bram on 16/07/25.
//

#ifndef DRIVEMOTORS_H
#define DRIVEMOTORS_H

#include "hardware/CustomServo.h"
#include "hardware/Motor.h"
#include "hardware/IRSensor.h"

class DriveMotors {
public:
    /**
     * Creates an object that represents a set of driving motors
     * @param lMotor the left motor
     * @param rMotor the right motor
     * @param lIRSensor the left IR sensor
     * @param rIRSensor the right IR sensor
     */
    DriveMotors(const Motor &lMotor, const Motor &rMotor, const IRSensor &lIRSensor, const IRSensor &rIRSensor);

    /**
     * Drives the robot with PID control with default parameters
     * @param speed the average speed to drive the robot at
     */
    void drivePID(int speed);

    /**
     * Drives the robot with PID control, with variable parameters
     * @param speed the average speed to drive the robot at
     * @param proportional the proportional term for the PID
     * @param derivative the derivative term for the PID
     */
    void drivePID(int speed, double proportional, double derivative);

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

protected:
    int leftMotorDirection = 1;
    int rightMotorDirection = 1;
    int currentSpeed = 0;
    Motor leftMotor;
    Motor rightMotor;
    IRSensor leftIRSensor;
    IRSensor rightIRSensor;
    int _distance = 0;
    int _last_distance = 0;
    int _qDist = 0;
    int _mDist = 0;
    int proportional = 0;
    int derivative = 0;
    int kProp = 500;
    int kDeriv = 10;
    int maxSpeed = 4095;
    int minSpeed = 0;
    int ctrl = 0;
    int leftReading = 0;
    int rightReading = 0;
    bool leftOnTape = 1;
    bool rightOnTape = 1;
    int lastOnTape = 0;
    int leftThreshold = 2500;
    int  rightThreshold = 2500;

    /**
     * calculates the current distance from the tape
     * @return the distance from the tape, either 0, +/- 1, or +/- 5
     */
    int distToTape();
};
#endif //DRIVEMOTORS_H

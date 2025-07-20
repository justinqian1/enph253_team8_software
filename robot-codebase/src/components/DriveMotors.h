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
    DriveMotors(Motor leftMotor, Motor rightMotor);
    void drive(int speed, int direction);
    void stop();
    void driveStraight(int speed, int direction);
    void driveLeftMotor(int speed, int direction);
    void driveRightMotor(int speed, int direction);

private:
    int currentDirection;
    int currentSpeed;
    Motor leftMotor;
    Motor rightMotor;
};
#endif //DRIVEMOTORS_H

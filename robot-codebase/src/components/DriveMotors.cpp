//
// Created by bram on 16/07/25.
//

#include "DriveMotors.h"
#include "hardware/Motor.h"

DriveMotors::DriveMotors(Motor &lMotor, Motor &rMotor, IRSensor &lIRSensor,
                         IRSensor &rIRSensor) : leftMotor(lMotor), rightMotor(rMotor), leftIRSensor(lIRSensor),
                                                      rightIRSensor(rIRSensor) {
}
/*
void DriveMotors::drivePID(int speed) {
    _last_distance = _distance;
    _distance = distToTape();

    if (_last_distance != _distance) {
        _qDist = _mDist;
        _mDist = 1;
    }

    proportional = kProp * _distance;
    derivative = (int) ((float) kDeriv * (float) (_distance - _last_distance) / (float) (_qDist + _mDist));
    // i+=ki*distance;
    ctrl = (int) (proportional + derivative);
    _mDist++;

    leftMotor.driveMotor(constrain(speed - ctrl, minSpeed, maxSpeed), HIGH);
    rightMotor.driveMotor(constrain(speed + ctrl, minSpeed, maxSpeed), HIGH);
    leftReading = leftIRSensor.read();
    rightReading = rightIRSensor.read();
}
*/
void DriveMotors::drivePID(int speed, int kp,
                           int kd) {
    _last_distance = _distance;
    _distance = distToTape();

    if (_last_distance != _distance) {
        _qDist = _mDist;
        _mDist = 1;
    }

    proportional = kp * _distance;
    derivative = (int) ((float) kd * (float) (_distance - _last_distance) / (float) (_qDist + _mDist));
    // i+=ki*distance;
    ctrl = proportional + derivative;
    _mDist++;

    leftMotor.driveMotor(constrain(speed - ctrl, minSpeed, maxSpeed), HIGH);
    rightMotor.driveMotor(constrain(speed + ctrl, minSpeed, maxSpeed), HIGH);
    leftReading = leftIRSensor.read();
    rightReading = rightIRSensor.read();
    Serial.print("Left reading");
    Serial.println(leftReading);
    Serial.print("Right reading:");
    Serial.println(rightReading);
    Serial.print("Left speed:");
    Serial.println(constrain(speed - ctrl, minSpeed, maxSpeed));
    Serial.print("Right speed:");
    Serial.println(constrain(speed + ctrl, minSpeed, maxSpeed));
}

void DriveMotors::stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
}

void DriveMotors::driveStraight(int speed, int direction) {
    driveLeftMotor(speed, direction);
    driveRightMotor(speed, direction);
}

void DriveMotors::driveLeftMotor(int speed, int direction) {
    leftMotor.driveMotor(speed, direction);
}

void DriveMotors::driveRightMotor(int speed, int direction) {
    rightMotor.driveMotor(speed, direction);
}


/**
 * distToTape - calculates the distance to the tape based on the IR sensor readings
 *
 * @return an integer indicating the robot's distance from the tape, either +/-5, +/-1, or 0 (on the line)
 */
int DriveMotors::distToTape()
{
    int dist = 0;
    leftOnTape = leftIRSensor.readThreshold(leftThreshold); // adc1 ch6 = pin 34
    rightOnTape = rightIRSensor.readThreshold(rightThreshold); // adc1 ch7 = pin 35
    if (leftOnTape && rightOnTape)
    {
        dist = 0;
    }
    else if (!leftOnTape && rightOnTape)
    {
        // only right sensor is on tape -> robot is to the left
        dist = -1;
        lastOnTape = 1;
    }
    else if (leftOnTape && !rightOnTape)
    {
        // only left sensor is on tape -> robot is to the left
        dist = 1;
        lastOnTape = -1;
    }
    else if (!leftOnTape && !rightOnTape && lastOnTape == 1)
    {
        // neither on tape but right was last one on tape
        dist = -5;
    }
    else
    {
        // neither on tape but left was last one on tape
        dist = 5;
    }
    return dist;
}
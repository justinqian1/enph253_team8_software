//
// Created by bram on 16/07/25.
//

#include "DriveMotors.h"
#include "hardware/Motor.h"

DriveMotors::DriveMotors(const Motor &lMotor, const Motor &rMotor, const IRSensor &lIRSensor,
                         const IRSensor &rIRSensor) : leftMotor(lMotor), rightMotor(rMotor), leftIRSensor(lIRSensor),
                                                      rightIRSensor(rIRSensor) {
}

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

void DriveMotors::drivePID(int speed, double proportional,
                           double derivative) : kProp(proportional), kDeriv(derivative) {
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

void DriveMotors::stop() {
    leftMotor.stop();
    rightMotor.stop();
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




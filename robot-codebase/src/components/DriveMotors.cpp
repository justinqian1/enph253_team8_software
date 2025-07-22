//
// Created by bram on 16/07/25.
//

#include "DriveMotors.h"
#include "hardware/Motor.h"

DriveMotors::DriveMotors(const Motor& lMotor, const Motor& rMotor) : leftMotor(lMotor), rightMotor(rMotor){
}

void DriveMotors::drivePID(int speed) {

}

void DriveMotors::drivePID(int speed, double proportional, double derivative, double integral) {

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




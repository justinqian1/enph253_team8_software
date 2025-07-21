//
// Created by bram on 16/07/25.
//

#include "DriveMotors.h"
#include "hardware/Motor.h"

DriveMotors::DriveMotors(const Motor& lMotor, const Motor& rMotor) : leftMotor(lMotor), rightMotor(rMotor){
}

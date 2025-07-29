//
// Created by bram on 16/07/25.
//

#ifndef CLAW_H
#define CLAW_H

#include <Arduino.h>
#include "hardware/Motor.h"
#include "hardware/CustomServo.h"
#include "hardware/HallSensor.h"
class Claw
{
public:
    Claw(CustomServo &rotation, CustomServo &grab, Motor &carriage, Motor &extension, HallSensor &hall);
    void zeroAll();
    void grab();
    void extend(int distance);
    void rise(int height);
    void setTurretAngle(int angle);
    void rotate(int angle);
    //blows up the robot
    void explode();
    //throws the pet
    void throwPet();

private:
    CustomServo rotationServo;
    CustomServo grabServo;
    Motor carriageMotor;
    Motor extensionMotor;
    HallSensor hallSensor;
    int currentAngle = 0;
    int height = 0;
    int extension = 0;
};
#endif //CLAW_H

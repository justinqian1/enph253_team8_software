//
// Created by bram on 16/07/25.
//

#ifndef CLAW_H
#define CLAW_H

#include <Arduino.h>
#include "hardware/Motor.h"
#include "hardware/CustomServo.h"
#include "hardware/HallSensor.h"
#include "hardware/RotaryEncoder.h"
#include "constants.h"
class Claw
{
public:
    Claw(CustomServo &rotation, CustomServo &grab, Motor &carriage, Motor &extension, HallSensor &hall, RotaryEncoder &rotary);
    void setupLimitSwitches(uint8_t carriageLow, uint8_t carriageHigh, uint8_t clawNotExtended, uint8_t clawExtended);
    void zeroAll();
    void grab();
    void open();
    void extend(bool direction);
    void extend(bool direction, int distance);
    void raise (bool up);
    void rotateTo(int angle);
    void rotateBy(int angle);
    int getExtensionPosition();
    //blows up the robot
    void explode();
    //throws the pet
    void throwPet();

private:
    CustomServo &rotationServo;
    CustomServo &grabServo;
    Motor &carriageMotor;
    Motor &extensionMotor;
    HallSensor &hallSensor;
    RotaryEncoder &rotaryEncoder;
    int currentAngle = 0;
    int height = 0;
    int extension = 0;
    bool clawFullyExtended = false; 
    bool clawFullyRetracted = true;
    uint8_t carriageLowSwitch = 0;
    uint8_t carriageHighSwitch = 0;
    uint8_t clawRetractedSwitch = 0;
    uint8_t clawExtendedSwitch = 0;
    uint8_t switchToPoll = 0;

};
#endif //CLAW_H

//
// Created by bram on 16/07/25.
//

#include "Claw.h"
#define OUT
#define IN


Claw::Claw(CustomServo &rotation, CustomServo &grab, Motor &carriage, Motor &extension,
           HallSensor &hall, RotaryEncoder &rotary) : rotationServo(rotation), grabServo(grab), carriageMotor(carriage),
                               extensionMotor(extension), hallSensor(hall), rotaryEncoder(rotary) {
    // constructor
}

void Claw::setupLimitSwitches(uint8_t carriageLow, uint8_t carriageHigh, uint8_t clawNotExtended, uint8_t clawExtended) {
    carriageLowSwitch = carriageLow;
    pinMode(carriageLow, INPUT_PULLUP);
    carriageHighSwitch = carriageHigh;
    pinMode(carriageHIGH, INPUT_PULLUP);
    clawRetractedSwitch = clawNotExtended;
    pinMode(clawNotExtended, INPUT_PULLUP);
    pinMode(clawExtended, INPUT_PULLUP);
}
void Claw::zeroAll() {
    // zero code here
}

void Claw::grab() {
    grabServo.rotateTo(clawClosedPos);
}

void Claw::open() {
    grabServo.rotateTo(clawOpenPos);
}

void Claw::extend(bool direction) {
    if (direction) 
    {
        clawFullyRetracted = false;
        extensionMotor.driveForward(clawExtSpeed);
        //finish here
    } else {
        //finish here
    }
}
void Claw::extend(bool direction, int distance) {

}

void Claw::raise(bool up) {}

void Claw::rotateTo(int angle) {
    rotationServo.rotateTo(angle);
}

void Claw::rotateBy(int angle) {
    rotationServo.rotateBy(angle);
}

int Claw::getExtensionPosition() {
    return rotaryEncoder.read();
}

void Claw::explode() {
    Serial.println("Now why would you do that to our poor robot?");
}

void Claw::throwPet() {

}
//
// Created by bram on 16/07/25.
//

#include "Claw.h"

Claw::Claw(CustomServo &rotation, CustomServo &grab, Motor &carriage, Motor &extension,
           HallSensor &hall) : rotationServo(rotation), grabServo(grab), carriageMotor(carriage),
                               extensionMotor(extension), hallSensor(hall) {
    // constructor
}

void Claw::zeroAll() {
    // zero code here
}

void Claw::grab() {


}

void Claw::extend(int distance) {


}

void Claw::rise(int height) {}

void Claw::setTurretAngle(int angle) {
    rotationServo.rotateTo(angle);
}

void Claw::rotate(int angle) {
    rotationServo.rotateBy(angle);
}

void explode() {
    Serial.println("Now why would you do that to our poor robot?");
}

void throwPet() {

}
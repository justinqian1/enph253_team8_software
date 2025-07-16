//
// Created by bram on 16/07/25.
//

#include "CustomServo.h"

CustomServo::CustomServo(int pin) : servoPin(pin), servoPosition(0) {}
CustomServo::CustomServo(int pin, int position) : servoPin(pin), servoPosition(position) {}




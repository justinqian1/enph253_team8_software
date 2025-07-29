//
// Created by bram on 29/07/25.
//

#include "RotaryEncoder.h"

#include <esp32-hal-gpio.h>

RotaryEncoder::RotaryEncoder(int pinA, int pinB) :  pinA(pinA), pinB(pinB) {
    instance = this;
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    attachInterrupt(pinA, handleInterrupt, CHANGE);
}

int RotaryEncoder::read() {
    return rotaryPosition;
}

void RotaryEncoder::update() {
    int mostSignificantBit = digitalRead(pinA);
    int leastSignificantBit = digitalRead(pinB);
    int bitEncodedValue = (mostSignificantBit << 1) | leastSignificantBit;
    if (bitEncodedValue != lastEncodedBitValue) {
        int bothEncoded = (lastEncodedBitValue  << 2) | bitEncodedValue;
        rotaryPosition = rotaryPosition + lookupTable[bothEncoded & 0x0F];
    }
    lastEncodedBitValue = bitEncodedValue;
}

void IRAM_ATTR RotaryEncoder::handleInterrupt() {
    if (instance) {
        instance->update();
    }
}

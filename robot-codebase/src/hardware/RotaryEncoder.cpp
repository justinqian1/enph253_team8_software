//
// Created by bram on 29/07/25.
//

#include "RotaryEncoder.h"

#include <esp32-hal-gpio.h>

RotaryEncoder* RotaryEncoder::instance = nullptr;
constexpr int RotaryEncoder::lookupTable[16];
RotaryEncoder::RotaryEncoder(int _pinA, int pinB) :  _pinA(_pinA), pinB(pinB) {
    instance = this;
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    attachInterrupt(_pinA, handleInterrupt, CHANGE);
}

int RotaryEncoder::read() {
    return rotaryPosition;
}

void RotaryEncoder::update() {
    int mostSignificantBit = digitalRead(_pinA);
    int leastSignificantBit = digitalRead(pinB);
    int bitEncodedValue = (mostSignificantBit << 1) | leastSignificantBit;
    if (bitEncodedValue != lastEncodedBitValue) {
        int bothEncoded = (lastEncodedBitValue  << 2) | bitEncodedValue;
        rotaryPosition = rotaryPosition + lookupTable[bothEncoded & 0x0F];
    }
    lastEncodedBitValue = bitEncodedValue;
}

void IRAM_ATTR RotaryEncoder::handleInterrupt() {
    instance->update();
    
}

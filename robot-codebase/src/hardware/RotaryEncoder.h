//
// Created by bram on 29/07/25.
//

#ifndef ROTARYENCODER_H
#define ROTARYENCODER_H

#include "driver/gpio.h"


class RotaryEncoder {
public:
    RotaryEncoder(int pinA, int pinB);
    RotaryEncoder(int pinA, int pinB, int min, int max);
    int read();

    void update();

    static void IRAM_ATTR handleInterrupt();

private:
    int pinA;
    int pinB;
    int min = -200;
    int max = 200;

};
#endif //ROTARYENCODER_H

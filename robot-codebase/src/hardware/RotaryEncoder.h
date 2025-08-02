//
// Created by bram on 29/07/25.
//

#ifndef ROTARYENCODER_H
#define ROTARYENCODER_H

#include "driver/gpio.h"


class RotaryEncoder {
public:
    RotaryEncoder(int _pinA, int pinB);
    int read();

    void update();

    static void IRAM_ATTR handleInterrupt();

private:
    int _pinA;
    int pinB;
    int minVal = -200;
    int maxVal = 200;
    int rotaryPosition = 0;
    int lastEncodedBitValue = 0x0;
    static RotaryEncoder* instance;
    static constexpr int lookupTable[] = {0,-1,1,0,
                                          1,0,0,-1,
                                          -1,0,0,1,
                                          0,1,-1,0};
};
#endif //ROTARYENCODER_H

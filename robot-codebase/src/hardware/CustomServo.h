//
// Created by bram on 16/07/25.
//

#ifndef SERVO_H
#define SERVO_H
#include "interfaces/servo_interface.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include <Arduino.h>

class CustomServo : public servo_interface
{
    public:
    CustomServo(int pin, int channel);
    CustomServo(int pin, int channel, int position);
    CustomServo(int pin, int channel, int position, int period);
    CustomServo(int pin, int channel, int position, int period, unsigned long min, unsigned long max);
    int getPin();
    int getPosition();
    void setAngle(int angle) override;
    void setAngle(int angle, int time) override;

    protected:
    int servoPin;
    int servoPosition;
    int periodHertz;
    int pwmChannel;
    static constexpr int maxDuty = (1 << 16) - 1;
    unsigned long minPulse;
    unsigned long maxPulse;
    int pulseLength(int pos);
    double dutyCycle(int length);
};
#endif //SERVO_H

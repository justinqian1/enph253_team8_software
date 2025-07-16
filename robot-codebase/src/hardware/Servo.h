//
// Created by bram on 16/07/25.
//

#ifndef SERVO_H
#define SERVO_H
#include "interfaces/servo_interface.h"

class Servo : public servo_interface
{
    public:
    explicit Servo(int pin);
    int getPin();
    int getPosition();
    void setAngle(int angle) override;
    void setAngle(int angle, int speed) override;

    private:
    int servoPin;
    int servoPosition;
};
#endif //SERVO_H

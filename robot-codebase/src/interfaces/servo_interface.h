//
// Created by bram on 16/07/25.
//

#ifndef SERVO_INTERFACE_H
#define SERVO_INTERFACE_H

class servo_interface
{
    public:
    virtual void rotateTo(int angle);
    virtual void rotateTo(int angle, int time);
    virtual ~servo_interface() = default;
};
#endif //SERVO_INTERFACE_H

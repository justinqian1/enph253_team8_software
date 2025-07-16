//
// Created by bram on 16/07/25.
//

#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

class motor_interface
{
    public:
    virtual void driveMotor(int speed, int direction);
    virtual void driveForward(int speed);
    virtual void driveReverse(int speed);
    virtual void stop();
    virtual ~motor_interface() = default;

};
#endif //MOTOR_INTERFACE_H

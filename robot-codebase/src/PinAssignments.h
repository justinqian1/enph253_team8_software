//
// Created by bram on 20/07/25.
//

#ifndef PINASSIGNMENTS_H
#define PINASSIGNMENTS_H


// PWM Channels

constexpr int leftPwmChannel = 0;
constexpr int rightPwmChannel = 1;
constexpr int carriagePWMChannel = 2;
constexpr int clawExtPWMChannel = 3;

// ESP32 pins
constexpr int pwmOut1 = 20; // outputs the pwm channel according to ledcAttachPin
constexpr int dirOut1 = 21;
constexpr int pwmOut2 = 22;
constexpr int dirOut2 = 19;
constexpr int irSensorLeft = 9;
constexpr int irSensorRight = 35;
constexpr int SG90Pin = 14;
constexpr int DSPin = 12;
constexpr int MG996RPin = 13;
constexpr int basketSwitch = 25;
constexpr int RXPin = 3; // I'm moving some pins around just for code simplicity but these can change later <-- NEED TO BE CHANGED, NOT IDEAL FOR UART
constexpr int TXPin = 1; // same as above
constexpr int startSwitch = 39;
constexpr int vertClawLOW = 26;
constexpr int vertClawHIGH = 32;
constexpr int horiClawLOW = 33;
constexpr int horiClawHIGH = 27;
constexpr int carriageMotorPWM = 5;
constexpr int carriageMotorDir = 10;
constexpr int clawExtMotorPWM =  8;
constexpr int clawExtMotorDir = 7;
constexpr int rotaryEncoderPinA = 2;
constexpr int rotaryEncoderPinB = 15;

// constants
constexpr int thresholdL = 1800;
constexpr int thresholdR = 1800;
constexpr int maxSpeed = 4000; // set a max pwm output
constexpr int minSpeed = 0;    // set a min pwm output
constexpr int homeSpeed = 600; // set a motor speed for the homing sequence
constexpr int angleThreshold = 75; // angle at which to initiate pick up sequence
constexpr int angleForward = 90;

// camera params
constexpr int img_size = 320;
constexpr float horizontal_fov = 62.2; //degrees

// misc consexpr
constexpr pcnt_unit_t PCNT_UNIT = PCNT_UNIT_0;

#endif //PINASSIGNMENTS_H

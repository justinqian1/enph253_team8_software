//
// Created by bram on 20/07/25.
//
#include "driver/pcnt.h"
#ifndef PINASSIGNMENTS_H
#define PINASSIGNMENTS_H

#include "driver/pcnt.h"

// PWM Channels

constexpr int leftPwmChannel = 6;
constexpr int rightPwmChannel = 7;
constexpr int carriageHeightPWMChannel = 2;
constexpr int clawExtPWMChannel = 3;
constexpr int carriageServoPWMChannel = 4;

// ESP32 pins
constexpr int pwmOut1 = 8; // outputs the pwm channel according to ledcAttachPin
constexpr int dirOut1 = 7;
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
constexpr int thresholdL = 3100;
constexpr int thresholdR = 3100;
constexpr int maxSpeed = 2500; // set a max pwm output
constexpr int minSpeed = 0;    // set a min pwm output
constexpr int homeSpeed = 600; // set a motor speed for the homing sequence
constexpr int carriageSpeed = 2000;

// for driving
constexpr int defaultKProp = 400; // kp and kd for driving pid control
constexpr int defaultKDeriv = 500;
constexpr int pwmFreq = 500;
constexpr int dir1 = 1;
constexpr int dir2 = 0;

//thresholds for pick up
constexpr int angleThreshold=75;
constexpr int clawCenterThreshold=20; //px from center
constexpr double areaThresholdForPickup=5000.0;

// misc cv params
constexpr int imgSize=320;
constexpr double horizontal_fov=62.2;
constexpr int angleForward=90;
constexpr int pwmChannel=0;

// misc consexpr
constexpr pcnt_unit_t PCNT_UNIT = PCNT_UNIT_0;

// hall sensor 
#ifndef CONSTANTS_H
#define CONSTANTS_H

#define HALL_SENSOR_PIN 35
#define HALL_VOLTAGE_REF 3.3
#define MAGNET_THRESHOLD_VOLTAGE 1.5

#endif //PINASSIGNMENTS_H


#endif //PINASSIGNMENTS_H

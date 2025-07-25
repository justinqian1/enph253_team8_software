//
// Created by bram on 20/07/25.
//
#include "driver/pcnt.h"
#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "driver/pcnt.h"

// PWM Channels

constexpr int leftPwmChannelFwd = 0;
constexpr int leftPwmChannelBwd = 1;
constexpr int rightPwmChannelFwd = 2;
constexpr int rightPwmChannelBwd = 3;
constexpr int carriageHeightPWMChannel = 4;
constexpr int clawExtPWMChannel = 5;
constexpr int carriageServoPWMChannel = 6;
constexpr int clawClosingPWMChannel = 7;

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
constexpr int startSwitch = 26;
constexpr int carriageLOW = 39;
constexpr int carriageHIGH = 32;
constexpr int horiClawLOW = 33;
constexpr int horiClawHIGH = 27;
constexpr int carriageMotorPWM = 5;
constexpr int carriageMotorDir = 10;
constexpr int clawExtMotorPWM =  8;
constexpr int clawExtMotorDir = 7;
constexpr int rotaryEncoderPinA = 2;
constexpr int rotaryEncoderPinB = 15;
constexpr int extraGND = 4;
constexpr int hallSensorPin = 36;

// constants
constexpr int thresholdL = 1300;
constexpr int thresholdR = 1800;
constexpr int maxSpeed = 4000; // set a max pwm output
constexpr int minSpeed = 0;    // set a min pwm output
constexpr int homeSpeed = 600; // set a motor speed for the homing sequence

// for driving
constexpr int defaultKProp = 500; // kp and kd for driving pid control
constexpr int defaultKDeriv = 550;
constexpr int pwmFreq = 500;
constexpr int dir1 = 0;
constexpr int dir2 = 1;

//thresholds for pick up
constexpr int angleThreshold=75;
constexpr int clawCenterThreshold=20; //px from center
constexpr double areaThresholdForPickup=5000.0;

// misc cv params
constexpr int imgSize=320;
constexpr double horizontal_fov=62.2;
constexpr int pwmChannel=0;

// carriage/servo setup
constexpr int servoFreq = 50;
constexpr int servoMinDuty = 500;
constexpr int servoMaxDuty = 2500;
constexpr int carriageForwardPos=90;

constexpr float encoderCountToInches = 0.05;

// extra motor speeds
constexpr int clawExtSpeed=2000;
constexpr int carriageSpeed = 2000;


// misc consexpr
constexpr pcnt_unit_t PCNT_UNIT = PCNT_UNIT_0;

// hall sensor 
constexpr double hallVoltageRef = 3.3;
constexpr double magnetThresholdVoltage = 1.5;

#endif //PINASSIGNMENTS_H

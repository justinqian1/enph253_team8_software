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
constexpr int carriageHeightPwmChannelUp = 4;
constexpr int carriageHeightPwmChannelDown = 5;
constexpr int clawExtPwmChannelExt = 6;
constexpr int clawExtPwmChannelRet = 7;
constexpr int carriageServoPwmChannel = 8;
constexpr int clawClosingServoPwmChannel = 9;

// ESP32 pins
constexpr int pwmOut1 = 8; // outputs the pwm channel according to ledcAttachPin
constexpr int dirOut1 = 7;
constexpr int pwmOut2 = 22;
constexpr int dirOut2 = 19;
constexpr int irSensorLeft = 9;
constexpr int irSensorRight = 35;
constexpr int SG90Pin = 14;
//constexpr int DSPin = 12;
constexpr int MG996RPin = 12;
constexpr int basketSwitch = 25;
constexpr int RXPin = 3; // I'm moving some pins around just for code simplicity but these can change later <-- NEED TO BE CHANGED, NOT IDEAL FOR UART
constexpr int TXPin = 1; // same as above
constexpr int startSwitch = 39;
constexpr int carriageLOW = 32;
constexpr int carriageHIGH = 25;
constexpr int clawExtendedSwitch = 33;
constexpr int clawRetractedSwitch = 26;
constexpr int carriageMotorPWM = 5;
constexpr int carriageMotorDir = 10;
constexpr int clawExtMotorPWM =  8;
constexpr int clawExtMotorDir = 7;
constexpr int rotaryEncoderPinA = 2;
constexpr int rotaryEncoderPinB = 15;
constexpr int extraGND = 4;
constexpr int hallSensorPin = 36;

// general constants
constexpr int pwmFreq = 500;

// driving related constants
constexpr int thresholdL = 1400;
constexpr int thresholdR = 2200;
constexpr int defaultSpeed = 1800;
constexpr int maxSpeed = 3000; // set a max pwm output
constexpr int minSpeed = 500;    // set a min pwm output
constexpr int homeSpeed = 600; // set a motor speed for the homing sequence

// for driving
constexpr int defaultKProp = 0; // kp and kd for driving pid control
constexpr int defaultKDeriv = 0;
constexpr int dir1 = 0;
constexpr int dir2 = 1;

//thresholds for pick up
constexpr int angleThreshold=75;
constexpr int clawCenterThreshold=20; //px from center
constexpr double areaThresholdForPickup=5000.0;

// misc cv params
constexpr int imgSize=320;
constexpr double horizontal_fov=62.2;

// carriage/servo setup
constexpr int servoFreq = 50;
constexpr int servoMinDuty = 500;
constexpr int servoMaxDuty = 2500;
constexpr int carriageForwardPos=120;
constexpr int carriageMaxLeftPos=0;
constexpr int carriageMaxRightPos=(int)(180.0*MG996RMultiplier);
constexpr double MG996RMultiplier = 1.316;

// limit switch related
constexpr int limitSwitchActiveThreshold = 2048;

// extra motor speeds
constexpr int clawExtSpeed=2000;
constexpr int carriageSpeed = 2000;

// SG90 (claw closing)
constexpr int clawOpenPos = 0;
constexpr int clawClosedPos = 90;

// misc consexpr
constexpr pcnt_unit_t PCNT_UNIT = PCNT_UNIT_0;

// limit switch stuff
enum SwitchHit : uint8_t {
    NONE = 0,
    CARRIAGE_LOW_SWITCH = 1,
    CARRIAGE_HIGH_SWITCH = 2,
    CLAW_EXT_SWITCH = 3,
    CLAW_RET_SWITCH = 4
};
constexpr uint32_t minSwitchID=1;
constexpr uint32_t maxSwitchID=4;
constexpr int switchPollFrequency = 20;

// hall sensor 
constexpr double hallVoltageRef = 3.3;
constexpr double magnetThresholdVoltage = 1.5;

#endif //PINASSIGNMENTS_H

//
// Created by bram on 20/07/25.
//
#include "driver/pcnt.h"
#ifndef CONSTANTS_H
#define CONSTANTS_H

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
constexpr int leftDriveFwdPin =20; // outputs the pwm channel according to ledcAttachPin
constexpr int leftDriveBwdPin = 21;
constexpr int rightDriveFwdPin = 19;
constexpr int rightDriveBwdPin = 22;
constexpr int carriageUpPin = 5;
constexpr int carriageDownPin = 10;
constexpr int clawExtPin =  7;
constexpr int clawRetPin = 8;

constexpr int irSensorLeft = 9;
constexpr int irSensorRight = 35;
constexpr int SG90Pin = 14;
//constexpr int DSPin = 12;
constexpr int MG996RPin = 12; // ds1 on esp
constexpr int basketSwitch = 25;
constexpr int RXPin = 3; // I'm moving some pins around just for code simplicity but these can change later <-- NEED TO BE CHANGED, NOT IDEAL FOR UART
constexpr int TXPin = 1; // same as above
constexpr int startSwitch = 39;
constexpr int carriageLOW = 32;
constexpr int carriageHIGH = 25;
constexpr int clawExtendedSwitch = 33;
constexpr int clawRetractedSwitch = 26;

//temp rotary encoder pins
constexpr int rotaryA = 0;
constexpr int rotaryB = 4;

// general constants
constexpr int pwmFreq = 500;

// driving related constants
constexpr int thresholdL = 2000;
constexpr int thresholdR = 2000;
constexpr int defaultSpeed = 3600;
constexpr int maxSpeed = 4095; // set a max pwm output
constexpr int minSpeed = 800;    // set a min pwm output
constexpr int homeSpeed = 600; // set a motor speed for the homing sequence

// for driving
constexpr int defaultKProp = 1000; // kp and kd for driving pid control
constexpr int defaultKDeriv = 1200;
constexpr int dir1 = 0;
constexpr int dir2 = 1;

//thresholds for pick up
constexpr int angleThreshold=75;
constexpr int clawCenterThreshold=30; //px from center
constexpr double areaThresholdForPickup=3500.0;

// misc cv params
constexpr int imgSize=320;
constexpr int imgCenter=160;
constexpr double horizontal_fov=62.2;

// turret/servo setup
constexpr int servoFreq = 50;
constexpr int servoMinDuty = 500;
constexpr int servoMaxDuty = 2500;
constexpr double MG996RMultiplier = 1.316;
constexpr int turretForwardPos=90;
constexpr int turretMaxLeftPos=0;
constexpr int turretMaxRightPos=(int)(180.0*MG996RMultiplier);

// limit switch related
constexpr int limitSwitchActiveThreshold = 2048;

// extra motor speeds
constexpr int clawExtSpeed=2000;
constexpr int carriageSpeed = 4095;

// SG90 (claw closing)
constexpr int clawOpenPos = 180;
constexpr int clawClosedPos = 0;

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

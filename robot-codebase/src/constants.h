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
constexpr int leftDriveFwdPin =22; // outputs the pwm channel according to ledcAttachPin
constexpr int leftDriveBwdPin = 19;
constexpr int rightDriveFwdPin = 21;
constexpr int rightDriveBwdPin = 20;
constexpr int carriageUpPin = 5;
constexpr int carriageDownPin = 10;
constexpr int clawExtPin = 7;
constexpr int clawRetPin = 8;

constexpr int irSensorLeft = 9;
constexpr int irSensorRight = 35;
constexpr int SG90Pin = 12; //orig=14
//constexpr int DSPin = 12; //orig = 12
constexpr int MG996RPin = 13; // orig=13
constexpr int basketSwitch = 25;
constexpr int RXPin = 3; // I'm moving some pins around just for code simplicity but these can change later <-- NEED TO BE CHANGED, NOT IDEAL FOR UART
constexpr int TXPin = 1; // same as above
constexpr int startSwitch = 39;
constexpr int carriageLOW = 33; 
constexpr int carriageHIGH = 26; 
constexpr int clawExtendedSwitch = 25;
constexpr int clawRetractedSwitch = 32;

//temp rotary encoder pins
constexpr int rotaryA = 37;
constexpr int rotaryB = 38;

// general constants
constexpr int pwmFreq = 500;

// driving related constants
constexpr int thresholdL = 3300;
constexpr int thresholdR = 3500; 
constexpr int defaultSpeed = 2400;
constexpr int maxSpeed = 4095; // set a max pwm output
constexpr int minSpeed = 1000;    // set a min pwm output
constexpr int minDriveSpeed = 1200;
constexpr int homeSpeed = 600; // set a motor speed for the homing sequence

// for driving
constexpr int defaultKProp = 600; // kp and kd for driving pid control
constexpr int defaultKDeriv = 900;
constexpr int dir1 = 0;
constexpr int dir2 = 1;

//thresholds for pick up
constexpr int defaultAngleThreshold=90;
constexpr int defaultStopDriveThreshold=95;
constexpr double clawCenterThreshold=5.0; //angle from center
constexpr double areaThresholdForPickup=2000.0;

// misc cv params
constexpr int imgSize=320;
constexpr int imgCenter=160;
constexpr double horizontal_fov=62.2;
constexpr int maxLineLength=40;

// turret/servo setup
constexpr int servoFreq = 50;
constexpr int servoMinDuty = 500;
constexpr int servoMaxDuty = 2400;
constexpr double MG996RMultiplier = 2.0;
constexpr int turretForwardPos=160;
constexpr int turretMaxLeftPos=turretForwardPos-55;
constexpr int turretMaxRightPos=360;

// limit switch related
constexpr int limitSwitchActiveThreshold = 2048;

// extra motor speeds
constexpr int clawExtSpeed=2400;
constexpr int carriageDownSpeed = 3500;
constexpr int carriageUpSpeed = 3500;

// SG90 (claw closing)
constexpr int clawOpenPos = 0;
constexpr int clawClosedPos = 75;

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

enum ClawPosition : uint8_t {
    FULL_RETRACT = 0,
    DROPOFF_RETRACT = 1,
    DEFAULT_RETRACT = 2,
    FULL_EXTEND = 3
};
constexpr int clawDefaultRetractTime = 240; // ms
constexpr int clawDropoffRetractTime = 300; // ms

//HARDCODING!!
constexpr int timeBeforePetDrop = 5000; // ms, time between resuming driving and dropping off first pet
constexpr int pet1AngleThreshold = 28; //degrees; actual = 27.7
constexpr int pet1StopDriveThreshold = 33; //degrees
constexpr int turretPosAfterFirstDrop = turretForwardPos-15; //degrees
constexpr int defaultSpeed2 = 2000;

// hall sensor 
constexpr double hallVoltageRef = 3.3;
constexpr double magnetThresholdVoltage = 1.5;

#endif //PINASSIGNMENTS_H

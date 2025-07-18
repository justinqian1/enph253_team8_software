//
// Created by bram on 17/07/25.
//

#include <Arduino.h>
#include "driver/ledc.h"
#include "driver/adc.h"

// pins numbers
constexpr int pwmOut1 = 20;
constexpr int dirOut1 = 21;
constexpr int pwmOut2 = 22;
constexpr int dirOut2 = 19;

// pwm channel defs
constexpr int leftPwmChannel = 0;
constexpr int rightPwmChannel = 1;

// constants
constexpr unsigned long deadTime = 5;
constexpr int testSpeed1 = 2000;
constexpr int testSpeed2 = 4095;
constexpr int switchTime = 2000;
// variables
int currentDirection = 0;
int currentSpeed = 9;

// task handles
TaskHandle_t drive_task_handle = NULL;

/**
 * drives the motors in a specified direction and a given speed, allotting dead time to prevent shoot through
 * @param speed the speed at which to drive the motors from 0 (min) to 4095 (max)
 * @param direction the direction in which to drive the motors
 */
void driveMotors(int speed, int direction)
{
    if (direction == currentDirection)
    {
        if (speed == currentSpeed)
        {
            // do nothing
        } else
        {
            ledcWrite(leftPwmChannel, speed);
            ledcWrite(rightPwmChannel, speed);
        }
    } else
    {
        ledcWrite(leftPwmChannel, 0);
        ledcWrite(rightPwmChannel, 0);
        currentDirection = direction;
        digitalWrite(dirOut1, direction);
        digitalWrite(dirOut2, direction);
        vTaskDelay(deadTime / portTICK_PERIOD_MS);
        ledcWrite(leftPwmChannel, speed);
        ledcWrite(rightPwmChannel, speed);
    }
}

/**
 * this task endlessly drives the motors, switching between two speeds and directions
 * @param parameters no parameters
 */
void drive_task (void * parameters){
    int driveSpeed = testSpeed1;
    int driveDirection = 0;
    while (1)
    {
        driveMotors(driveSpeed, driveDirection);

        // drive for switchTime milliseconds
        vTaskDelay(switchTime / portTICK_PERIOD_MS);

        // swap direction and speed
        driveDirection = (driveDirection == 0) ? 1 : 0;

        driveSpeed = (driveSpeed == testSpeed1) ? testSpeed2 : testSpeed1;
    }
}

void setup()
{
    //intialize all states as low to avoid shoot through (direction pins don't matter to much but it's just in case)
    digitalWrite(dirOut1, LOW);
    digitalWrite(dirOut2, LOW);


    // intialize PWM channels
    ledcSetup(leftPwmChannel, 250, 12); // middle number: duty cycle resolution in hz
    ledcSetup(rightPwmChannel, 250, 12);
    ledcAttachPin(pwmOut1, leftPwmChannel);
    ledcAttachPin(pwmOut2, rightPwmChannel); // both motors controlled by same pwm channel
    ledcWrite(leftPwmChannel,0);
    ledcWrite(rightPwmChannel,0);

    // setup pins
    pinMode(pwmOut1, OUTPUT);
    pinMode(pwmOut2, OUTPUT);
    pinMode(dirOut1, OUTPUT);
    pinMode(dirOut2, OUTPUT);

    xTaskCreate(
        drive_task,
        "Driving",
        2000,
        nullptr,
        1,
        &drive_task_handle
    );
}

void loop()
{
// nothing to see here
}
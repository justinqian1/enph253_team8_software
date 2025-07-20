//
// Created by bram on 17/07/25.
//

#include <Arduino.h>
#include "driver/ledc.h"
#include "driver/adc.h"
#include "hardware/CustomServo.h"

// pins numbers
constexpr int pwmOut1 = 20;
constexpr int dirOut1 = 27;
constexpr int pwmOut2 = 22;
constexpr int dirOut2 = 19;

// pwm channel defs
constexpr int leftPwmChannel = 0;
constexpr int rightPwmChannel = 1;

// constants
constexpr unsigned long deadTime = 5;
constexpr int testSpeed1 = 2000;
constexpr int testSpeed2 = 4095;
constexpr int switchTime = 4000;
// variables
int currentDirection = 0;
int currentSpeed = 0;

// task handles
TaskHandle_t drive_test_task_handle = NULL;
TaskHandle_t servo_test_task_handle = NULL;

// servos
constexpr int servoPin = 21;
constexpr int pwmChannel = 3;
CustomServo testServo(servoPin, pwmChannel, 180);
void test_set_angle(int pos) {
    testServo.setAngle(pos);
}
/**
 * drives the motors in a specified direction and a given speed, allotting dead time to prevent shoot through
 * @param speed the speed at which to drive the motors from 0 (min) to 4095 (max)
 * @param direction the direction in which to drive the motors
 */
void driveMotors(int speed, int direction)
{
    if (direction == currentDirection)
    {
        ledcWrite(leftPwmChannel, speed);
        ledcWrite(rightPwmChannel, speed);

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
void drive_test_task (void * parameters){
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
        Serial.println("Switching directions!");
    }
}

void servo_test_task (void * parameters) {
    while (1) {
        test_set_angle( 90);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        test_set_angle(120);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}
void presetup()
{
    /*
    Serial.begin(115200);
    while(!Serial);
    Serial.println("TESTING");
    adc1_config_width(ADC_WIDTH_BIT_12);

    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_12); // ir sensor inputs (pin 32)
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_12); // pin 33
    adc2_config_channel_atten(ADC2_CHANNEL_7, ADC_ATTEN_DB_12); // pin 27 = p_pot (to be removed later)
    adc2_config_channel_atten(ADC2_CHANNEL_6, ADC_ATTEN_DB_12); // pin 14 = d_pot
    //intialize all states as low to avoid shoot through (direction pins don't matter to much but it's just in case)
    digitalWrite(dirOut1, LOW);
    digitalWrite(dirOut2, LOW);


    // intialize PWM channels
    ledcSetup(leftPwmChannel, 250, 12); // middle number: duty cycle resolution in hz
    ledcSetup(rightPwmChannel, 250, 12);
    ledcAttachPin(pwmOut1, leftPwmChannel);
    ledcAttachPin(pwmOut2, rightPwmChannel);
    ledcWrite(leftPwmChannel,2000);
    ledcWrite(rightPwmChannel,2000);

    // setup pins
    pinMode(dirOut1, OUTPUT);
    pinMode(dirOut2, OUTPUT);

    Serial.println("Setup Complete!!");
    /*
    
    xTaskCreate(
        drive_test_task,
        "Driving",
        2000,
        nullptr,
        0,
        &drive_test_task_handle
    );*/
    xTaskCreate(
        servo_test_task,
        "Servo Rotating",
        2000,
        nullptr,
        1,
        &servo_test_task_handle
    );
}


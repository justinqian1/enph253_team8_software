#include <Arduino.h>
#include "driver/ledc.h"
#include "driver/adc.h"
#include <ESP32Servo.h>
#include "driver/gpio.h"
#include <HardwareSerial.h>
#include "driver/pcnt.h"
#include "constants.h"
#include "hardware/CustomServo.h"
#include "hardware/Motor.h"
#include "components/RobotWheels.h"
#include "tasks.h"

void test_drive(void *parameters) {
    RobotWheels* r = static_cast<RobotWheels*>(parameters);
    for (;;) {
        r -> driveLeftMotor(3000,0);
        //Serial.println("Driving!");
        vTaskDelay(pdMS_TO_TICKS(1000));
        r -> driveStraight(2000,1);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
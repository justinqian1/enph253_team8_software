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
#include "globals.h"
#include "functions.h"


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

void detect_task(void *parameters)
{
    // detection code for determining pet location
    while (1) {
        if (Serial2Pi.available()) {
            String line = Serial2Pi.readStringUntil('\n');
            if (line=="[SYSTEM MESSAGE] RESET") {
                resetVars();
                //rotationTested=false;
                Serial2Pi.printf("System message 'RESET' received\n");
            } else if (line.length()>1) {
                // pet in visual range AND large enough (done on pi)
                sscanf(line.c_str(), "%lf,%lf,%lf,%lf", &petX, &petY, &petW, &petH);
                Serial2Pi.printf("ESP received: x=%.2lf y=%.2lf w=%.2lf h=%.2lf\n", petX, petY, petW, petH);

                double petArea = petW*petH;
                int currentAngle=MG996R->getPosition();
                Serial2Pi.printf("servo angle: %d\n",currentAngle);

                //check if pet big enough for pickup
                closeEnough = petArea > areaThresholdForPickup;           

                // check if angle is correct (off forward direction by at least 75 deg)
                anglePastThreshold = (currentAngle < carriageForwardPos - angleThreshold ||
                                  currentAngle > carriageForwardPos + angleThreshold);

                //rotate turret
                double rotate_const=1.0;
                if (abs((int)petX-imgSize/2) > clawCenterThreshold) { //tolerance of 20; center of pet in pixels 140-180 is good enough
                    MG996R->rotateBy((int)(angleToCenter(petX)*rotate_const));
                    clawCentered=false;
                } else {
                    clawCentered = true;
                }

                // check if ready for pickup
                Serial2Pi.printf("Claw centered: %d\n",clawCentered);
                Serial2Pi.printf("Pet close enough: %d\n",closeEnough);
                Serial2Pi.printf("Angle past threshold: %d\n",anglePastThreshold);

                if (clawCentered && closeEnough && anglePastThreshold) {
                    // arm is at nearly 90 degree angle -> initiate pick up
                    robot->stop();
                    //vTaskSuspend(drive_handle);
                    pickUpPet();      
                } else {
                    // not close enough - update speed
                    int tempSpeedCeiling = (int)(-0.25*petArea+2300.0); // arbitrary function for now, decreases speed as pet draws closer
                    int currentSpeed = speed;
                    tempSpeedCeiling=max(tempSpeedCeiling,1000); // make sure speed is positive
                    speed=min(currentSpeed,tempSpeedCeiling);
                    Serial2Pi.printf("robot speed: %d\n",speed);
                }
                //Serial.printf("x=%.2f y=%.2f w=%.2f h=%.2f\n", pet_x, pet_y, pet_w, pet_h);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
#include <Arduino.h>
#include "driver/ledc.h"
#include "driver/adc.h"
#include <ESP32Servo.h>
#include "driver/gpio.h"
#include <HardwareSerial.h>
#include "constants.h"
#include "hardware/CustomServo.h"
#include "hardware/Motor.h"
#include "components/RobotWheels.h"
#include "tasks.h"

// TRUE IF RUNNING ON COMP SURFACE, FALSE IF TESTING
bool run = false;
// global variables and task handles

TaskHandle_t drive_handle = nullptr;
TaskHandle_t home_handle = nullptr;
TaskHandle_t full_turn_handle = nullptr;
TaskHandle_t detect_handle = nullptr;
TaskHandle_t read_uart_handle = nullptr;
TaskHandle_t drop_first_pet_handle = nullptr;

// initialize serial port for Pi communication

HardwareSerial Serial2Pi(0); // for UART 0

// robot properties/location
volatile int speed = defaultSpeed;    // average speed
int petsPickedUp = 0;
bool rotationTested=false; // for testing
volatile bool pickupNext=false;

volatile bool carriageHigh = false;
volatile bool carriageLow = false;
volatile bool clawFullyExtended = false;
volatile bool clawPartRetracted = false;
volatile bool clawFullyRetracted = false;

//booleans for pick up
bool closeEnough = false;
bool clawCentered = false;
bool anglePastThreshold = false;
bool anglePastStopDriveThreshold = false;

// other vars
unsigned long startTime = 0;
uint32_t reverseMultiplier = 0.3; // percentage speed of average speed for driving backwards

// UART info
typedef struct {
    float petX;
    float petArea;
    float angleFromCenter;
} PetInfo;

QueueHandle_t petInfoQueue;

// servos
CustomServo* clawCloseServo;
CustomServo* turretServo;

//  motor declarations
Motor* rightMotor;//(rightPwmChannelFwd, 20, rightPwmChannelBwd, 21);
Motor* leftMotor;//(leftPwmChannelFwd, 8, leftPwmChannelBwd, 7);
IRSensor* leftIRSensor;//(ADC1_CHANNEL_6);
IRSensor* rightIRSensor;//(ADC1_CHANNEL_7);
RobotWheels* robot;//(leftMotor, rightMotor, leftIRSensor, rightIRSensor);

Motor* carriageMotor;
Motor* clawExtMotor;

// function declarations
void resetVars();
void moveCarriage(bool up);
void extendClaw (uint8_t position);
void closeClaw(bool close);
void pickUpPet();
void dropPetInBasket();
void prepareForNextPickup();
bool checkSwitchHit(uint32_t switch_id);
bool pollSwitch(uint32_t switch_id);
void home();

bool heightsForPickup[6] = {true, true, true, true, true, true}; //false = low, true = high
bool pickupSide[6] = {false, true, true, true, true, false}; // false = left, true = right
double petDistToTape[6] = {10.0, 10.0, 14.0, 14.0, 14.0, 14.0}; //distances in inches from tape

// TEST PARAMETERS

/**
 * resets pet detection related variables after pickup
 */
void resetVars() {
    if (carriageHigh) {
        turretServo->rotateTo(turretForwardPos);
    }
    closeEnough = false;
    clawCentered = false;
    anglePastThreshold = false;
    anglePastStopDriveThreshold=false;
    pickupNext = false;
    speed=defaultSpeed;
}

void setupLimitSwitches() {
    pinMode(carriageLOW, INPUT_PULLUP);
    pinMode(carriageHIGH, INPUT_PULLUP);
    pinMode(clawExtendedSwitch, INPUT_PULLUP);
    pinMode(clawRetractedSwitch, INPUT_PULLUP);
}

/** 
 * changes carriage height
 * @param up, true if moving up and false if moving down
 */
void moveCarriage(bool up) {
    // if already moved to target position
    if (up && carriageHigh ||
       !up && carriageLow) {
        return;
    }

    // move carriage
    if (up) {
        carriageMotor->driveForward(carriageUpSpeed);
    } else {
        carriageMotor->driveReverse(carriageDownSpeed);
    } 
    Serial2Pi.println(up ? "Moving carriage upwards" : "Moving carriage downwards");
    uint32_t switchToPoll;
    up ? switchToPoll = CARRIAGE_HIGH_SWITCH : switchToPoll = CARRIAGE_LOW_SWITCH;
    pollSwitch(switchToPoll); // poll switches, which returns when the switch is hit
}

void extendClaw (uint8_t position) {
    // if already moved to target position
    if (position == FULL_EXTEND && clawFullyExtended ||
        position == PART_RETRACT && clawPartRetracted ||
        position == FULL_RETRACT && clawFullyRetracted) {
        return;
    }

    // move claw
    switch (position) {
        case FULL_EXTEND:
            Serial2Pi.println("Claw extending");
            clawFullyRetracted = false;
            clawPartRetracted = false;
            clawExtMotor->driveForward(clawExtSpeed);
            pollSwitch(CLAW_EXT_SWITCH);
            break;
        case FULL_RETRACT:
            Serial2Pi.println("Claw retracting");
            clawFullyExtended = false;
            clawPartRetracted = false;
            clawExtMotor->driveReverse(clawExtSpeed);
            pollSwitch(CLAW_RET_SWITCH);
            break;
        case PART_RETRACT:
            if (!clawFullyExtended) {
                extendClaw(FULL_EXTEND);
            }
            Serial2Pi.println("Claw partial retracting");
            clawFullyExtended = false;
            clawFullyRetracted = false;
            clawExtMotor->driveReverse(clawExtSpeed);
            delay(clawPartRetractTime); // allow motor to run
            clawPartRetracted = true;
            clawExtMotor->stopMotor();
            Serial2Pi.println("Claw partially retracted");
    }
}

void closeClaw(bool close) {
    if (close) {
        clawCloseServo->rotateTo(clawClosedPos);
        Serial.println("claw closing");
    } else {
        clawCloseServo->rotateTo(clawOpenPos);
        Serial.println("claw opening");
    }
}

/**
 * picks up pet 
 */
void pickUpPet() {   
    // get carriage to right height (should be already done though)
    bool targetHeight = heightsForPickup[petsPickedUp];
    if (targetHeight && !carriageHigh) {
        moveCarriage(true);
    } else if (!targetHeight && !carriageLow) {
        moveCarriage(false);
    }

    extendClaw(FULL_EXTEND); // may hardcode distances for each pet
    // before this, the claw should be at partial retraction
    delay(20);
    closeClaw(true);
    delay(2000); //time for the pet to be grabbed
    petsPickedUp++;
    Serial2Pi.printf("Pet picked up!\n");

    if (run && petsPickedUp==0) { // hardcoding for pet #1 on the surface
        moveCarriage(true);
        turretServo->rotateTo(180);
        delay(1000);
        speed=defaultSpeed;
        vTaskResume(drive_handle);
        xTaskNotifyGive(drop_first_pet_handle);
    } else { // all other cases
        dropPetInBasket(); // START DROP SEQUENCE
    }
}

void dropPetInBasket() {
    if (!carriageHigh) { // make sure carriage is high
        moveCarriage(true);
    }
    
    extendClaw(PART_RETRACT); // retract claw partially for dropoff

    int servoRotateTime;
    if (turretServo->getPosition() < turretForwardPos) {
        servoRotateTime=2500;
    } else {
        servoRotateTime = 1000;
    }
    turretServo->rotateTo(turretMaxRightPos,servoRotateTime); //rotate to max angle over some amount of time
    closeClaw(false); // open claw
    delay(2000); // give time to drop pet

    extendClaw(FULL_RETRACT); // retract after drop
    turretServo->rotateTo(270); // rotate back to right-facing position
    extendClaw(PART_RETRACT); // go back to default partial retraction position
    if(petsPickedUp < 6) {
        prepareForNextPickup();
    } else { // full turn - may change the conditional
        speed=defaultSpeed;
        Serial2Pi.println("Turning around");
        //xTaskNotifyGive(full_turn_handle);
    }
}

void prepareForNextPickup() {
    Serial2Pi.println("Preparing for next pickup");
    //pickupSide[petsPickedUp] ? turretServo->rotateTo(turretForwardPos-45) : turretServo->rotateTo(turretForwardPos+45);
    turretServo->rotateTo(turretForwardPos+30); // face rightwards after pickup
    moveCarriage(heightsForPickup[petsPickedUp]);
    // now claw should be open, carriage should be set for next pickup and rotated properly
    speed=defaultSpeed;
    vTaskResume(drive_handle);
}

void testRotation() {    
    turretServo->rotateTo(180);
    delay(2000);

    turretServo->rotateBy(-45);
    delay(2000);

    turretServo->rotateBy(-45);
    delay(2000);

    turretServo->rotateBy(-10);
    delay(2000);
/*
    // turretServo->rotateTo(180);
    // delay(2000);

    // turretServo->rotateBy(90);
    // delay(2000);

    // turretServo->rotateBy(60);
    // delay(2000);

    // turretServo->rotateTo(180);
    // delay(2000);
    */
}

/**
 * checks for a specific limit switch being hit
 * @param switch_id the switch id to check
 *                  MUST be between 1-4
 */
bool checkSwitchHit(uint32_t switch_id) {
    bool result;
    switch(switch_id) {
        case CARRIAGE_HIGH_SWITCH:
            result = analogRead(carriageHIGH) > limitSwitchActiveThreshold;
            if (result) {
                carriageHigh=true;
                carriageLow=false;
                carriageMotor->stopMotor();
                Serial2Pi.println("Carriage high switch hit");
            }
            break;
        case CARRIAGE_LOW_SWITCH:
            result = analogRead(carriageLOW) > limitSwitchActiveThreshold;
            if (result) {
                carriageHigh=false;
                carriageLow=true;
                carriageMotor->stopMotor();
                Serial2Pi.println("Carriage low switch hit");
            }
            break;
        case CLAW_EXT_SWITCH:
            result = analogRead(clawExtendedSwitch) > limitSwitchActiveThreshold;
            if (result) {
                clawFullyExtended=true;
                clawFullyRetracted=false;
                clawExtMotor->stopMotor();
                Serial2Pi.println("Claw full extension switch hit");
            }
            break;
        case CLAW_RET_SWITCH:
            result = analogRead(clawRetractedSwitch) > limitSwitchActiveThreshold;
            if (result) {
                clawFullyExtended=false;
                clawFullyRetracted=true;
                clawExtMotor->stopMotor();
                Serial2Pi.println("Claw full retraction switch hit");
            }
            break;
        default:
            Serial2Pi.println("Error: unknown switch ID");
            return true;
    }
    return result;
}

bool pollSwitch(uint32_t switch_id) {
    int count=0;
    
    if (switch_id < minSwitchID || switch_id > maxSwitchID) {
        // switchToPoll value invalid
        Serial2Pi.print("Error: cannot poll switch ");
        Serial2Pi.println(switch_id);
        return false;
    }
    //poll switch
    Serial2Pi.print("Polling switch ");
    Serial2Pi.println(switch_id);
    while(!checkSwitchHit(switch_id)) {
        if (count % (2000/switchPollFrequency) == 0) {
            Serial2Pi.println("Still waiting for switch to hit..."); // prints every 2s
        }
        count++;
        delay(switchPollFrequency);
    }
    return true; // when switch hits
}

/**
 * clears uart buffer so info from pictures taken during pickup sequence is discarded
 * done after pickups
 */
void clearUART() {
    while (Serial2Pi.available()) {
        Serial2Pi.read();  // clears uart input buffer
    }
    PetInfo clearPetInfo;
    while (xQueueReceive(petInfoQueue, &clearPetInfo, 0) == pdTRUE) {} // clears queue
}

/**
 * Runs the homing sequence for the robot
 */
void home()
{
    /**
     * Code for homing sequence to run on startup, including:
     * Homing DC motors using limit switches (2 motors)
     * Setting all servo motor positions to home positions
     */
    
    //extends claw, then moves carriage down then back up, then retracts claw
    extendClaw(FULL_EXTEND);
    turretServo->rotateTo(turretForwardPos);
    delay(500);
    moveCarriage(true);
    moveCarriage(false); // carriage low at start
    extendClaw(PART_RETRACT); // claw at full retraction at start

    clawCloseServo->rotateTo(clawOpenPos);
}

// freeRTOS tasks

/**
 * this task operates the main driving system of the robot, including PID control. It also includes a hard-coded stop 
 * condition if the time reaches 90 seconds, at which point it signals the full_turn task to execute at max priority.
 * @param parameters no parameters for this task
 */
void drive_task(void *parameters)
{
    if (run) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    for (;;) {
        robot->drivePID(speed);
        if (/*run && */millis() - startTime > 7000)
        {
            startTime = millis();
            xTaskNotifyGive(&full_turn_handle);

        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

/**
 * this is a one-time task that activates the homing sequence, after which it puts the robot in idle mode. It also
 * terminates itself upon completion.
 * @param parameters no parameters for this task
 */
void home_task(void *parameters)
{
    // homing sequence, to be run once at startup and then deleted
    startTime=millis();
    home();

    // start driving and then delete this task as it will not occur again.
    xTaskNotifyGive(&drive_handle);
    xTaskNotifyGive(&read_uart_handle);
    vTaskDelete(NULL);
}

void read_uart_task(void *parameters) {
    char line[maxLineLength];
    int lineIdx=0;
    if(run) {
        ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    }
    while (1) {
        while (Serial2Pi.available()) {
            char c = Serial2Pi.read();
            if (c == '\n') {
                line[lineIdx] = '\0';
                // strip junk chars
                int j = 0;
                for (int i = 0; i < lineIdx; i++) {
                    if ((unsigned char)line[i] >= 32 && (unsigned char)line[i] <= 126) {
                        line[j++] = line[i];
                    }
                }
                line[j] = '\0';

                PetInfo petInfo;
                if (strcmp(line,"[SYSTEM MESSAGE] RESET")==0) {
                    resetVars();
                    turretServo->rotateTo(turretForwardPos);
                    Serial2Pi.printf("System message 'RESET' received\n");
                } else {
                    // Serial2Pi.printf("Raw line bytes: ");
                    // for (int i = 0; i < strlen(line); i++) {
                    //     Serial2Pi.printf("%02X ", (unsigned char)line[i]);
                    // }
                    // Serial2Pi.println();
                    int numParsed = sscanf(line, "%f,%f,%f", &petInfo.petX, &petInfo.petArea, &petInfo.angleFromCenter);
                    //Serial2Pi.printf("Parsed %d values from line: %s\n", numParsed, line);

                    if (numParsed == 3) {
                        Serial2Pi.printf("ESP received: x=%.2f, area=%.2f, angle from center=%.2f\n", 
                            petInfo.petX, petInfo.petArea, petInfo.angleFromCenter);
                        xQueueOverwrite(petInfoQueue, &petInfo);
                    } else {
                        Serial2Pi.printf("Failed to parse line: %s\n", line);
                    }
                }
                lineIdx = 0; // reset buffer
                memset(line, 0, sizeof(line)); 
            } else {
                line[lineIdx++] = c;

            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/**
 * this task handles detection of pets through serial communication with the Raspberry Pi 5. It signals the home_claw_task
 * to update its position. It also signals when a pet is close enough to be picked up to operate the grab task.
 * @param parameters no parameters for this task
 */
void detect_task(void *parameters)
{
    // detection code for determining pet location
    PetInfo petInfo;
    while(1) {
        if (xQueueReceive(petInfoQueue,&petInfo,portMAX_DELAY)==pdPASS) {        
            int currentAngle=turretServo->getPosition();
            Serial2Pi.printf("servo angle: %d\n",currentAngle);

            //check if pet big enough for pickup
            closeEnough = petInfo.petArea > areaThresholdForPickup;

            // check if angle is correct (off forward direction by at least 80 deg)
            int angleThreshold, stopDriveThreshold;
            if (run && petsPickedUp == 0){
                angleThreshold = pet1AngleThreshold;
                stopDriveThreshold = pet1StopDriveThreshold;
            } else {
                angleThreshold = defaultAngleThreshold;
                stopDriveThreshold = defaultStopDriveThreshold;
            }
            anglePastThreshold = (currentAngle < turretForwardPos - angleThreshold ||
                                currentAngle > turretForwardPos + angleThreshold);

            anglePastStopDriveThreshold = (currentAngle < turretForwardPos - stopDriveThreshold ||
                                            currentAngle > turretForwardPos + stopDriveThreshold);

            // check if claw is centered on pet
            clawCentered = abs(petInfo.angleFromCenter) < clawCenterThreshold; 

            // check if ready for pickup
            if (clawCentered) {
                Serial2Pi.printf("Claw centered\n");
            }
            if(closeEnough) {
                Serial2Pi.printf("Pet close enough\n");
            }
            if (anglePastThreshold) {
                Serial2Pi.printf("Angle past threshold\n");
            }

            if (clawCentered && closeEnough && anglePastThreshold && pickupNext) {
                vTaskSuspend(read_uart_handle);
                turretServo->rotateBy((int)(round(petInfo.angleFromCenter)));
                vTaskDelay(400);
                Serial2Pi.printf("Initiating pickup!\n");
                // pickUpPet();

                Serial2Pi.printf("Pet picked up!\n");
                vTaskDelay(pdMS_TO_TICKS(3000)); // allow robot to start going again before detect task restarts
                speed=defaultSpeed;
                vTaskResume(drive_handle);

                pickupNext=false;
                clearUART();
                if(!(run && petsPickedUp==1)) {
                    vTaskResume(read_uart_handle);
                }
            } else if ((clawCentered && closeEnough && anglePastThreshold) ||
                        (closeEnough && anglePastStopDriveThreshold)) {
                Serial2Pi.printf("Pickup on next frame\n");
                pickupNext=true;
                robot->stop();
                vTaskSuspend(drive_handle);
            } else {
                // not close enough - update angle and speed
                float rotateKP=min(petInfo.petArea/5000.0,1.0);
                int rotationAmount = (int)(round(petInfo.angleFromCenter*rotateKP));
                Serial2Pi.printf("Rotating turret by %d\n",rotationAmount);
                turretServo->rotateBy(rotationAmount);
                int tempSpeedCeiling = (int)(-petInfo.petArea+4500.0); // arbitrary function for now, decreases speed as pet draws closer
                int currentSpeed = speed;
                tempSpeedCeiling=max(tempSpeedCeiling,minDriveSpeed); // make sure speed is positive
                speed=min(currentSpeed,tempSpeedCeiling);
                Serial2Pi.printf("Still driving. Robot speed: %d\n",speed);
            }
        }
    }

}

void drop_first_pet_task(void *parameters) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // wait until switch poll finishes
    vTaskDelay(timeBeforePetDrop);

    vTaskSuspend(drive_handle);

    turretServo->rotateTo(90);
    closeClaw(false);
    vTaskDelay(2000);
    turretServo->rotateTo(turretPosAfterFirstDrop);

    clearUART();
    vTaskResume(read_uart_handle);
    vTaskResume(drive_handle);
    vTaskDelete(NULL);
}

void full_turn_task(void *parameters) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    vTaskSuspend(&drive_handle);

    robot -> driveLeftMotor(4095,0);
    robot -> driveRightMotor(4095,1);
    vTaskDelay(1000);

    for(;;) {
        if (leftIRSensor->read() > thresholdL && rightIRSensor->read() > thresholdR)  {
            robot -> stop();
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    vTaskResume(&drive_handle);
}

void setup()
{
    // serial
    Serial2Pi.begin(115200, SERIAL_8N1, RXPin, TXPin);
    Serial2Pi.write("Hello from the ESP32!");
    petInfoQueue = xQueueCreate(1, sizeof(PetInfo));

    // motor/servo setup
    rightMotor = new Motor(rightPwmChannelFwd, rightDriveFwdPin, rightPwmChannelBwd, rightDriveBwdPin);
    leftMotor = new Motor(leftPwmChannelFwd, leftDriveFwdPin, leftPwmChannelBwd, leftDriveBwdPin);
    leftIRSensor = new IRSensor(ADC1_CHANNEL_6);
    rightIRSensor = new IRSensor(ADC1_CHANNEL_7);
    robot = new RobotWheels(*leftMotor, *rightMotor, *leftIRSensor, *rightIRSensor);
    carriageMotor = new Motor(carriageHeightPwmChannelUp,carriageUpPin,carriageHeightPwmChannelDown,carriageDownPin);
    clawExtMotor = new Motor(clawExtPwmChannelExt,clawExtPin,clawExtPwmChannelRet,clawRetPin);
    clawCloseServo = new CustomServo(SG90Pin, clawClosingServoPwmChannel, clawOpenPos, servoFreq, servoMinDuty, servoMaxDuty);
    turretServo = new CustomServo(MG996RPin,carriageServoPwmChannel, turretForwardPos, servoFreq, servoMinDuty, servoMaxDuty, MG996RMultiplier);

    // limit switches
    setupLimitSwitches();
    if (run) { 
        xTaskCreate(
            home_task,   // function to be run
            "Homing",   // description of task
            4096,          // bytes allocated to this stack
            NULL,          // parameters, dependent on function
            1,             // priority
            &home_handle // task handle
        );
        xTaskCreate(
            drop_first_pet_task,   // function to be run
            "Dropping first pet off",   // description of task
            1000,          // bytes allocated to this stack
            NULL,          // parameters, dependent on function
            1,             // priority
            &drop_first_pet_handle // task handle
        );
    }
    // xTaskCreate(
    //     detect_task,   // function to be run
    //     "Detecting",   // description of task
    //     4096,          // bytes allocated to this stack
    //     NULL,          // parameters, dependent on function
    //     1,             // priority
    //     &detect_handle // task handle
    // );
    // xTaskCreate(
    //     read_uart_task,   // function to be run
    //     "Read UART",   // description of task
    //     4096,          // bytes allocated to this stack
    //     NULL,          // parameters, dependent on function
    //     1,             // priority
    //     &read_uart_handle // task handle
    // );
    xTaskCreate(
        drive_task,   // function to be run
        "Driving",    // description of task
        4096,         // bytes allocated to this 
        NULL,         // parameters, dependent on function
        1,            // priority
        &drive_handle // task handle
    );
    xTaskCreate(
        full_turn_task,
        "Turning",
        4096,
        nullptr,
        6,
        &full_turn_handle
    );
}

void loop()
{
    // pollSwitch(CLAW_EXT_SWITCH);
    // pollSwitch(CLAW_RET_SWITCH);
    // pollSwitch(CARRIAGE_HIGH_SWITCH);
    // pollSwitch(CARRIAGE_LOW_SWITCH);
    // home();
    // turretServo->rotateTo(180);
    // delay(2000);
    // turretServo->rotateTo(360);
    // delay(2000);
    // testRotation();
    // pickUpPet();
    // delay(4000);
    // if(petsPickedUp > 5) {
    //     petsPickedUp=0;
    // }
    // Serial.println("Testing carriage");
    // moveCarriage(!carriageHigh);
    // Serial.print("Carriage position now ");
    // Serial.println(carriageHigh);
    // delay(1000);

    // closeClaw(true);
    // delay(1000); 
    // closeClaw(false);
    // delay(1000);

    // extendClaw(FULL_EXTEND);
    // delay(2000);
    // extendClaw(FULL_RETRACT);
    // delay(2000);
    // closeClaw(true);
    // delay(2000);
    // closeClaw(false);
    // delay(2000);
    // extendClaw(PART_RETRACT);
    // delay(5000);
        // PUT TEST CODE HERE

        // if (!rotationTested) {
            // turretServo->rotateTo(turretForwardPos);
            // moveCarriage(true);
            // turretServo->rotateTo(turretMaxRightPos);
            // testRotation();
            // rotationTested=true; 
        // }         
    // to be left empty, robot should run in the freeRTOS task scheduler
}

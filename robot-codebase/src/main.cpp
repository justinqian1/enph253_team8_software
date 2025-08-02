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
#include "functions.h"
#include "globals.h"

#define UP true
#define DOWN false
// TRUE IF RUNNING, FALSE IF TESTING
bool run = false;
// global variables and task handles

// TaskHandle_t drive_handle = nullptr;
// TaskHandle_t grab_handle = nullptr;
// TaskHandle_t home_handle = nullptr;
// TaskHandle_t raise_carriage_handle = nullptr;
// TaskHandle_t test_raise_carriage_handle = nullptr;
// TaskHandle_t poll_switch_handle = nullptr;
// TaskHandle_t full_turn_handle = nullptr;
// TaskHandle_t detect_handle = nullptr;
// TaskHandle_t idle_handle = nullptr;

// // initialize serial port for Pi communication

// HardwareSerial Serial2Pi(0); // for UART 0

// // robot properties/location
// volatile int speed = defaultSpeed;    // average speed
// int petsPickedUp = 0;
// bool rotationTested=false; // for testing

// volatile bool carriageHigh = false;
// volatile bool carriageLow = false;
// volatile bool clawFullyExtended = false;
// volatile bool clawFullyRetracted = false;

// // PID vars
// int distance = 0; // right = positive
// int last_distance = 0;
// int p = 0;
// int d = 0;
// int m = 0; // counts cycles since the reading last changed; more cycles = smaller d
// int q = 0; // equivalent to m_last (last previous value of m)
// int ctrl = 0;
// //int dir1 = 1;
// //int dir2 = 1;
// bool leftOnTape = 1;
// bool rightOnTape = 1;
// int leftVal = 0;
// int rightVal = 0;
// int leftSpeed = 0;
// int rightSpeed = 0;
// int lastOnTape = 0; // -1: left; 1: right

// // pet location vars
// double petX = 0;
// double petY = 0;
// double petW = 0;
// double petH = 0;

// //booleans for pick up
// bool closeEnough = false;
// bool clawCentered = false;
// bool anglePastThreshold = false;

// // other vars
 unsigned long startTime = 0;
// uint32_t reverseMultiplier = 0.3; // percentage speed of average speed for driving backwards
// int rotaryCounter = 0;
// int16_t rotaryMax = 0;
// int16_t rotaryMin = 0;

// // for limit switches
// volatile bool carriageSwitchHit = false;
volatile int rotaryPosition = 0;
volatile int lastEncodedBitValue  = 0;
constexpr int lookupTable[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
volatile int isrTrigger = 0;
volatile unsigned long lastTime = 0;

// // servos
// // for closing the claw
// CustomServo* SG90;//(SG90Pin, clawClosingServoPwmChannel, clawOpenPos, servoFreq, servoMinDuty, servoMaxDuty);

// // for rotating turret
// CustomServo* MG996R;//(MG996RPin,carriageServoPwmChannel, carriageForwardPos, servoFreq, servoMinDuty, servoMaxDuty, MG996RMultiplier);

// //  motor declarations
// Motor* rightMotor;//(rightPwmChannelFwd, 20, rightPwmChannelBwd, 21);
// Motor* leftMotor;//(leftPwmChannelFwd, 8, leftPwmChannelBwd, 7);
// Motor* carriageMotor;
// Motor* clawExtMotor;
// IRSensor* leftIRSensor;//(ADC1_CHANNEL_6);
// IRSensor* rightIRSensor;//(ADC1_CHANNEL_7);
// RobotWheels* robot;//(leftMotor, rightMotor, leftIRSensor, rightIRSensor);

// // Motor carriageMotor(carriageHeightPwmChannelUp,carriageUpPin,carriageHeightPwmChannelDown,carriageDownPin);
// // Motor clawExtMotor(clawExtPwmChannelExt,clawExtPin,clawExtPwmChannelRet,clawRetPin);

// // function declarations
// void resetVars();
// int distToTape();
// double angleToCenter(double petX);
// void drive(int avgSpeedInput);
// void configIRSensors();
// void moveCarriage(bool up);
// void extendClaw (bool outwards);
// void closeClaw(bool close);
// void pickUpPet();
// void dropPetInBasket();
// void prepareForNextPickup();
// bool checkSwitchHit(uint32_t switch_id);
// bool pollSwitch(uint32_t switch_id);
// void home();

// bool heightsForPickup[6] = {false, false, true, true, false, false}; //false = low, true = high
// bool pickupSide[6] = {false, true, true, true, true, false}; // false = left, true = right
// double petDistToTape[6] = {10.0, 14.0, 14.0, 14.0, 14.0, 14.0}; //distances in inches from tape

// TEST PARAMETERS

/**
 * resets pet detection related variables after pickup
//  */
// void resetVars() {
//     MG996R->rotateTo(carriageForwardPos);
//     closeEnough = false;
//     clawCentered = false;
//     anglePastThreshold = false;
//     speed=defaultSpeed;
// }

// /**
//  * calculates angle to center of pet. Note that the input image is flipped vertically.
//  * @param pet_x_coord center of pet's x coordinate
//  * @return angle between -31 (pet on very left of frame) to +31 (pet on very right of frame)
//  */
// double angleToCenter(double petX) {
//     double result= (petX-(double)imgSize/2)/(double)imgSize*horizontal_fov;
//     Serial2Pi.printf("Turret off by: %.2lf\n",result);
//     return result;
// }

// void setupLimitSwitches() {
//     pinMode(carriageLOW, INPUT_PULLUP);
//     pinMode(carriageHIGH, INPUT_PULLUP);
//     pinMode(clawExtendedSwitch, INPUT_PULLUP);
//     pinMode(clawRetractedSwitch, INPUT_PULLUP);
// }

// /** 
//  * changes carriage height
//  * @param up, true if moving up and false if moving down
//  */
// void moveCarriage(bool up) {
//     carriageMotor->driveMotor(carriageSpeed,up);
//     Serial2Pi.println(up ? "Moving carriage upwards" : "Moving carriage downwards");
//     uint32_t switchToPoll;
//     up ? switchToPoll = CARRIAGE_HIGH_SWITCH : switchToPoll = CARRIAGE_LOW_SWITCH;
//     pollSwitch(switchToPoll);
// }

// void extendClaw (bool outwards) {
//     uint32_t switchToPoll;
//     if (outwards) {
//         Serial2Pi.println("Claw extending");
//         clawFullyRetracted = false;
//         clawExtMotor->driveForward(clawExtSpeed);
//         switchToPoll = CLAW_EXT_SWITCH;
//     } else {
//         Serial2Pi.println("Claw retracting");
//         clawFullyExtended = false;
//         clawExtMotor->driveReverse(clawExtSpeed);
//         switchToPoll = CLAW_RET_SWITCH;
//     }
//     pollSwitch(switchToPoll);
// }

// void closeClaw(bool close) {
//     if (close) {
//         SG90->rotateTo(clawClosedPos);
//         Serial2Pi.println("claw closing");
//     } else {
//         SG90->rotateTo(clawOpenPos);
//         Serial2Pi.println("claw opening");
//     }
// }

// /**
//  * picks up pet
//  */
// void pickUpPet() {
//     vTaskSuspend(detect_handle);
//     bool targetHeight = heightsForPickup[petsPickedUp];
//     if (targetHeight && !carriageHigh) {
//         moveCarriage(UP);
//         //xTaskNotify(raise_carriage_handle, targetHeight, eSetValueWithOverwrite); // doesn't work rn since we need it to return
//     } else if (!targetHeight && !carriageLow) {
//         moveCarriage(DOWN);
//     }
//     extendClaw(true);
//     // receive input from hall effect
//     closeClaw(true);
//     delay(1000);
//     petsPickedUp++;
//     Serial2Pi.printf("Pet picked up!\n");
//     //dropPetInBasket();
//     return; // START DROP SEQUENCE
// }

// void dropPetInBasket() {
//     if (!carriageHigh) {
//         moveCarriage(UP);
//         //xTaskNotify(raise_carriage_handle, true, eSetValueWithOverwrite); // moves carriage up if it's low; DOESN'T WORK RN
//     }
    
//     extendClaw(false); // retract claw
//     MG996R->rotateTo(carriageMaxRightPos); //rotate to max angle
//     closeClaw(false); // open claw
//     delay(1500); // give time to drop pet
//     extendClaw(true); // re extend claw
//     prepareForNextPickup();
// }

// void prepareForNextPickup() {
//     Serial2Pi.println("Preparing for next pickup");
//     moveCarriage(UP); // move carriage up
//     pickupSide[petsPickedUp] ? MG996R->rotateTo(carriageForwardPos-45) : MG996R->rotateTo(carriageForwardPos+45);

//     extendClaw(false);
//     vTaskResume(detect_handle);
//     //vTaskResume(drive_handle);
// }

// void testRotation() {    
//     MG996R->rotateTo(90);
//     Serial2Pi.println("position set to 90");
//     delay(1500);

//     MG996R->rotateBy(90);
//     Serial2Pi.println("position should be 180");
//     delay(1500);

//     MG996R->rotateBy(-90);
//     Serial2Pi.println("position should be 90");
//     delay(1500);

//     MG996R->rotateBy(-90);
//     Serial2Pi.println("position should be 0");
//     delay(1500);

//     MG996R->rotateBy(90);
//     Serial2Pi.println("position should be 90");
//     Serial2Pi.println("test ended");
// }

// /**
//  * checks for a specific limit switch being hit
//  * @param switch_id the switch id to check
//  *                  MUST be between 1-4
//  */
// bool checkSwitchHit(uint32_t switch_id) {
//     bool result;
//     switch(switch_id) {
//         case CARRIAGE_HIGH_SWITCH:
//             result = analogRead(carriageHIGH) > limitSwitchActiveThreshold;
//             if (result) {
//                 carriageHigh=true;
//                 carriageLow=false;
//                 carriageMotor->stopMotor();
//                 //xTaskNotifyGive(raise_carriage_handle);
//                 Serial.println("Carriage high switch hit");
//             }
//             break;
//         case CARRIAGE_LOW_SWITCH:
//             result = analogRead(carriageLOW) > limitSwitchActiveThreshold;
//             if (result) {
//                 carriageHigh=false;
//                 carriageLow=true;
//                 carriageMotor->stopMotor();
//                 //xTaskNotifyGive(raise_carriage_handle);
//                 Serial.println("Carriage low switch hit");
//             }
//             break;
//         case CLAW_EXT_SWITCH:
//             result = analogRead(clawExtendedSwitch) > limitSwitchActiveThreshold;
//             if (result) {
//                 clawFullyExtended=true;
//                 clawFullyRetracted=false;
//                 clawExtMotor->stopMotor();
//                 Serial.println("Claw full extension switch hit");
//             }
//             break;
//         case CLAW_RET_SWITCH:
//             result = analogRead(clawRetractedSwitch) > limitSwitchActiveThreshold;
//             if (result) {
//                 clawFullyExtended=false;
//                 clawFullyRetracted=true;
//                 clawExtMotor->stopMotor();
//                 Serial.println("Claw full retraction switch hit");
//             }
//             break;
//         default:
//             Serial.println("Error: unknown switch ID");
//             return true;
//     }
//     return result;
// }

// bool pollSwitch(uint32_t switch_id) {
//     int count=0;
    
//     if (switch_id < minSwitchID || switch_id > maxSwitchID) {
//         // switchToPoll value invalid
//         Serial.print("Error: cannot poll switch ");
//         Serial.println(switch_id);
//         return false;
//     }
//     //poll switch
//     Serial.print("Polling switch ");
//     Serial.println(switch_id);
//     while(!checkSwitchHit(switch_id)) {
//         if (count % (1000/switchPollFrequency) == 0) {
//             Serial.println("Still waiting for switch to hit...");
//         }
//         count++;
//         delay(switchPollFrequency);
//     }
//     return true; // when switch hits
// }

// /**
//  * Runs the homing sequence for the robot
//  * NEEDS FULL REWRITE - COMMENTED OUT FOR NOW
//  */
// void home()
// {
//     /**
//      * Code for homing sequence to run on startup, including:
//      * Homing DC motors using limit switches (2 motors)
//      * Setting all servo motor positions to 0
//      */

//     // uint32_t switchHit;
//     /*
//     SG90Pos = 0;
//     DSPos = 0;
//     MG996RPos = 0;

//     SG90.write(SG90Pos);
//     DS.write(DSPos);
//     MG996R.write(MG996RPos);
//     */

//     // find limits of the claw
//     // driveAndreMotor(clawExtPwmChannelExt, clawExtPwmChannelRet, homeSpeed, 0);
//     // xTaskNotifyWait(0, 0xFFFFFFFF, &switchHit, portMAX_DELAY);
//     // if (switchHit == 3)
//     // {
//     //     pcnt_get_counter_value(PCNT_UNIT, &rotaryMin);
//     // }
//     // else if (switchHit == 4)
//     // {
//     //     pcnt_get_counter_value(PCNT_UNIT, &rotaryMax);
//     // }

//     // driveAndreMotor(clawExtPwmChannelExt, clawExtPwmChannelRet, homeSpeed, 1);
//     // xTaskNotifyWait(0, 0xFFFFFFFF, &switchHit, portMAX_DELAY);
//     // if (switchHit == 3)
//     // {
//     //     pcnt_get_counter_value(PCNT_UNIT, &rotaryMin);
//     // }
//     // else if (switchHit == 4)
//     // {
//     //     pcnt_get_counter_value(PCNT_UNIT, &rotaryMax);
//     // }
//     // stopMotor(clawExtPwmChannelExt,clawExtPwmChannelRet);

//     // driveAndreMotor(carriageHeightPwmChannelUp, carriageHeightPwmChannelDown, homeSpeed, 0);
//     // xTaskNotifyWait(0, 0xFFFFFFFF, &switchHit, portMAX_DELAY);
//     // if (switchHit == 1)
//     // {
//     //     stopMotor(carriageHeightPwmChannelUp,carriageHeightPwmChannelDown);
//     // }
//     // else if (switchHit == 2)
//     // {
//     //     driveAndreMotor(carriageHeightPwmChannelUp, carriageHeightPwmChannelDown, homeSpeed, 1);
//     //     xTaskNotifyWait(0, 0xFFFFFFFF, &switchHit, portMAX_DELAY);
//     //     stopMotor(carriageHeightPwmChannelUp, carriageHeightPwmChannelDown);
//     // }
// }

// Another ISR implementation for the start button to go (can also be a switch)
void IRAM_ATTR startButtonPressedISR()
{
    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(idle_handle, &hpw);
    portYIELD_FROM_ISR(&hpw);
}

void IRAM_ATTR carriageLowPressedISR()
{
    // BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // xTaskNotifyFromISR(lower_switch_handle, LOW_SWITCH, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
    // portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    // BaseType_t hpw = pdFALSE;
    // xTaskNotifyFromISR(home_handle, 0x01, eSetBits, &hpw);
    // portYIELD_FROM_ISR(&hpw);
}

void IRAM_ATTR carriageHighPressedISR()
{
    // BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // xTaskNotifyFromISR(upper_switch_handle, HIGH_SWITCH, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
    // portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    // BaseType_t hpw = pdFALSE;
    // xTaskNotifyFromISR(home_handle, 0x02, eSetBits, &hpw);
    // portYIELD_FROM_ISR(&hpw);
}

void IRAM_ATTR clawFullExtensionPressedISR()
{
    clawFullyExtended = true;
    // BaseType_t hpw = pdFALSE;
    // xTaskNotifyFromISR(home_handle, 0x03, eSetBits, &hpw);
    // portYIELD_FROM_ISR(&hpw);
}

void IRAM_ATTR clawFullRetractionPressedISR() {
    clawFullyRetracted = true;
}

void IRAM_ATTR encoderRead() {
    int mostSignificantBit = digitalRead(rotaryA);
    int leastSignificantBit = digitalRead(rotaryB); 
    int bitEncodedValue = (mostSignificantBit << 1) | leastSignificantBit;
    if (bitEncodedValue != lastEncodedBitValue) {
    int bothEncoded = (lastEncodedBitValue  << 2) | bitEncodedValue;
    rotaryPosition = rotaryPosition + lookupTable[bothEncoded & 0x0F];
    }
    lastEncodedBitValue = bitEncodedValue;

    isrTrigger++;
}

// freeRTOS tasks

/**
 * this task operates the main driving system of the robot, including PID control. It also includes a hard-coded stop c
 * condition if the time reaches 90 seconds, at which point it signals the full_turn task to execute at max priority.
 * @param parameters no parameters for this task
 */
void drive_task(void *parameters)
{

    //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // this creates an infinite loop, but it will be interrupted by other actions
    for (;;)
    {

       // drive(speed);
        //robot.drivePID(speed);
        Serial2Pi.printf("driving");

        // if (millis() - startTime > 90000)
        // {
        //     xTaskNotifyGive(&full_turn_handle);
        // }
        // the vTaskDelay function takes in ticks as a time measurement, so / portTick_PERIOD_MS converts to ms
        vTaskDelay(1000 / portTICK_PERIOD_MS);
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
    home();

    // start driving and then delete this task as it will not occur again.
    xTaskNotifyGive(&idle_handle);
    vTaskDelete(NULL);
}

/**
 * this task handles detection of pets through serial communication with the Raspberry Pi 5. It signals the home_claw_task
 * to update its position. It also signals when a pet is close enough to be picked up to operate the grab task.
 * @param parameters no parameters for this task
 */
// void detect_task(void *parameters)
// {
//     // detection code for determining pet location
//     while (1) {
//         if (Serial2Pi.available()) {
//             String line = Serial2Pi.readStringUntil('\n');
//             if (line=="[SYSTEM MESSAGE] RESET") {
//                 resetVars();
//                 //rotationTested=false;
//                 Serial2Pi.printf("System message 'RESET' received\n");
//             } else if (line.length()>1) {
//                 // pet in visual range AND large enough (done on pi)
//                 sscanf(line.c_str(), "%lf,%lf,%lf,%lf", &petX, &petY, &petW, &petH);
//                 Serial2Pi.printf("ESP received: x=%.2lf y=%.2lf w=%.2lf h=%.2lf\n", petX, petY, petW, petH);

//                 double petArea = petW*petH;
//                 int currentAngle=MG996R->getPosition();
//                 Serial2Pi.printf("servo angle: %d\n",currentAngle);

//                 //check if pet big enough for pickup
//                 closeEnough = petArea > areaThresholdForPickup;           

//                 // check if angle is correct (off forward direction by at least 75 deg)
//                 anglePastThreshold = (currentAngle < carriageForwardPos - angleThreshold ||
//                                   currentAngle > carriageForwardPos + angleThreshold);

//                 //rotate turret
//                 double rotate_const=1.0;
//                 if (abs((int)petX-imgSize/2) > clawCenterThreshold) { //tolerance of 20; center of pet in pixels 140-180 is good enough
//                     MG996R->rotateBy((int)(angleToCenter(petX)*rotate_const));
//                     clawCentered=false;
//                 } else {
//                     clawCentered = true;
//                 }

//                 // check if ready for pickup
//                 Serial2Pi.printf("Claw centered: %d\n",clawCentered);
//                 Serial2Pi.printf("Pet close enough: %d\n",closeEnough);
//                 Serial2Pi.printf("Angle past threshold: %d\n",anglePastThreshold);

//                 if (clawCentered && closeEnough && anglePastThreshold) {
//                     // arm is at nearly 90 degree angle -> initiate pick up
//                     robot->stop();
//                     //vTaskSuspend(drive_handle);
//                     pickUpPet();      
//                 } else {
//                     // not close enough - update speed
//                     int tempSpeedCeiling = (int)(-0.25*petArea+2300.0); // arbitrary function for now, decreases speed as pet draws closer
//                     int currentSpeed = speed;
//                     tempSpeedCeiling=max(tempSpeedCeiling,1000); // make sure speed is positive
//                     speed=min(currentSpeed,tempSpeedCeiling);
//                     Serial2Pi.printf("robot speed: %d\n",speed);
//                 }
//                 //Serial.printf("x=%.2f y=%.2f w=%.2f h=%.2f\n", pet_x, pet_y, pet_w, pet_h);
//             }
//         }
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }

void raise_carriage_task(void *parameters) {
    uint32_t direction; // encodes the direction of motion (1=up,0=down)

    while (1) {
        xTaskNotifyWait(0,0xFFFFFFFF,&direction,portMAX_DELAY); // Wait forever until ISR notifies

        moveCarriage(direction);
        xTaskNotify(poll_switch_handle,direction+1,eSetValueWithOverwrite); // start switch poll
        // high switch has id 2, low switch has id 1 hence the direction+1

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // wait until switch poll finishes
        Serial.println("Switch poll finished; raise carriage task exiting");
        // stopMotor(carriageHeightPwmChannelUp, carriageHeightPwmChannelDown);
        
        //xTaskNotifyGive(test_raise_carriage_handle);
    }
}

void test_raise_carriage_task(void *parameters) {
    uint32_t dir = 1;
    while (1) {
        // Send notify to raise_carriage_task to start movement
        xTaskNotify(raise_carriage_handle, dir, eSetValueWithOverwrite);
        Serial.println("Notified raise_carriage_task");

        // wait for test to be done
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        dir == 1 ? dir = 0 : dir = 1;
        Serial.println("Carriage test complete");
        Serial.println("carriageHigh: ");
        Serial.println(carriageHigh);
        // Wait before starting again
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void poll_switch_task(void *parameters) {
    uint32_t switchToPoll;
    while (1) {
        xTaskNotifyWait(0,0,&switchToPoll,portMAX_DELAY);
        int count = 0;
        // error check
        if (switchToPoll < minSwitchID || switchToPoll > maxSwitchID) {
            // switchToPoll value invalid
            Serial.print("Error: cannot poll switch ");
            Serial.println(switchToPoll);
            vTaskDelete(NULL);
            return;
        }

        // poll switch
        Serial.print("Polling switch ");
        Serial.println(switchToPoll);
        while (!checkSwitchHit(switchToPoll)) {
            if (count % (1000/switchPollFrequency) == 0) {
                Serial.println("Still waiting for switch to hit...");
            }
            count++;
            vTaskDelay(pdTICKS_TO_MS(switchPollFrequency));
        }
        Serial.println("Poll switch task exiting");
    }
}

/**
 * this task handles operating the servo attached to the rotating base to follow the nearest pet according to the Pi Cam.
 * It is constantly updated by the detect_task and runs independently of the driving function.
 * @param parameters no parameters for this task
 */
void home_claw_task(void *parameters)
{
    // code to turn camera to follow nearest pet
}
void idle_task(void *parameters)
{
    // initiate idling once homing is finished

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // idling until start button is pressed

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    xTaskNotifyGive(&drive_handle);

    startTime = millis();

    vTaskDelete(NULL);
}

// void test_drive(void *parameters) {
//     RobotWheels* r = static_cast<RobotWheels*>(parameters);
//     for (;;) {
//         r -> driveLeftMotor(3000,0);
//         //Serial.println("Driving!");
//         vTaskDelay(pdMS_TO_TICKS(1000));
//         r -> driveStraight(4095,1);
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }

void full_turn_test(void *parameters) {
    for(;;) {
        robot -> driveLeftMotor(4095,0);
        robot -> driveRightMotor(4095,1);
        vTaskDelay(1000);
        if (leftIRSensor->read() < thresholdL && rightIRSensor->read() < thresholdR)  {
            robot -> stop();
        }
        vTaskDelay(4000);
    }
}
// add more functions here

void setup()
{
    // put your setup code here, to run once:
    if (run) { 
        Serial.begin(9600);
        // WILL NEED FULL REWRITE
        /*
        // initialize UART connection to the Pi
        Serial2Pi.begin(115200, SERIAL_8N1, RXPin, TXPin);
        Serial2Pi.write("Hello from the ESP32!");

        // initialize basic pin connections

        // needs to be pull up for encoder to function properly (I think <-- TO BE TESTED)
        pinMode(rotaryA, INPUT_PULLUP);
        pinMode(rotaryB, INPUT_PULLUP);

        // initialize adc channels for pwm signals to operate drive
        adc1_config_width(ADC_WIDTH_BIT_12);

        adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_12); // ir sensor inputs (pin 32)
        adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_12); // pin 33
        adc2_config_channel_atten(ADC2_CHANNEL_7, ADC_ATTEN_DB_12); // pin 27 = p_pot (to be removed later)
        adc2_config_channel_atten(ADC2_CHANNEL_6, ADC_ATTEN_DB_12); // pin 14 = d_pot

        ledcSetup(leftPwmChannelFwd, 250, 12); // middle number: duty cycle resolution in hz
        ledcSetup(rightPwmChannelFwd, 250, 12);
        // ledcAttachPin(pwmOut1, leftPwmChannelFwd);
        // ledcAttachPin(pwmOut2, rightPwmChannelFwd);

        //ledcSetup(carriageHeightPWMChannel, 250, 12);
        //ledcAttachPin(carriageMotorPWM, carriageHeightPWMChannel);

        //ledcSetup(clawExtPWMChannel, 250, 12);
        //ledcAttachPin(clawExtMotorPWM, clawExtPWMChannel);

        // attach pins for ISRs

        pinMode(startSwitch, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(startSwitch), startButtonPressedISR, RISING);

        // starts the PCNT setup code
        // rotaryEncoderSetup();
        // create tasks associated with functions defined above
        // priorities are temporary and TBD
        xTaskCreate(
            drive_task,   // function to be run
            "Driving",    // description of task
            1000,         // bytes allocated to this ib_deps = madhephaestus/ESP32Servo@^3.0.8stack
            NULL,         // parameters, dependent on function
            1,            // priority
            &drive_handle // task handle
        );

        // high priority task since it occurs on startup
        xTaskCreate(
            home_task,   // function to be run
            "Homing",    // description of task
            1000,        // bytes allocated to this stack
            NULL,        // parameters, dependent on function
            5,           // priority
            &home_handle // task handle
        );

        xTaskCreate(
            detect_task,   // function to be run
            "Detecting",   // description of task
            1000,          // bytes allocated to this stack
            NULL,          // parameters, dependent on function
            2,             // priority
            &detect_handle // task handle
        );

        xTaskCreate(
            idle_task,   // function to be run
            "Idling",    // description of task
            1000,        // bytes allocated to this stack
            NULL,        // parameters, dependent on function
            5,           // priority
            &idle_handle // task handle
        );
        */
    }

    if (!run) {
        // Serial2Pi.begin(115200, SERIAL_8N1, RXPin, TXPin);
        // Serial2Pi.write("Hello from the ESP32!");
        // xTaskCreate(
        //     detect_task,   // function to be run
        //     "Detecting",   // description of task
        //     4096,          // bytes allocated to this stack
        //     NULL,          // parameters, dependent on function
        //     2,             // priority
        //     &detect_handle // task handle
        // );
        // xTaskCreate(
        //     drive_task,   // function to be run
        //     "Driving",    // description of task
        //     1000,         // bytes allocated to this ib_deps = madhephaestus/ESP32Servo@^3.0.8stack
        //     NULL,         // parameters, dependent on function
        //     1,            // priority
        //     &drive_handle // task handle
        // );
        Serial.begin(9600);
        rightMotor = new Motor(rightPwmChannelFwd, rightDriveFwdPin, rightPwmChannelBwd, rightDriveBwdPin);
        leftMotor = new Motor(leftPwmChannelFwd, leftDriveFwdPin, leftPwmChannelBwd, leftDriveBwdPin);
        carriageMotor = new Motor(carriageHeightPwmChannelUp, carriageUpPin, carriageHeightPwmChannelDown, carriageDownPin);
        clawExtMotor = new Motor(clawExtPwmChannelExt, clawExtPin, clawExtPwmChannelRet, clawRetPin);
        leftIRSensor = new IRSensor(ADC1_CHANNEL_6);
        rightIRSensor = new IRSensor(ADC1_CHANNEL_7);
        robot = new RobotWheels(*leftMotor, *rightMotor, *leftIRSensor, *rightIRSensor);
        SG90 = new CustomServo(SG90Pin, clawClosingServoPwmChannel, clawOpenPos, servoFreq, 544, 2400);
        MG996R = new CustomServo(MG996RPin,carriageServoPwmChannel, carriageForwardPos, servoFreq, servoMinDuty, servoMaxDuty, MG996RMultiplier);
        Serial.println("MG996R DEBUG");
        Serial.print("Pin: ");
        Serial.println(MG996R->getPin());

        ledcSetup(12,50,16);
        ledcAttachPin(26,12);
        // ledcSetup(0,pwmFreq, 12);
        // ledcSetup(1,pwmFreq, 12);
        // ledcSetup(2,pwmFreq,12);
        // ledcSetup(3,pwmFreq,12);
        // ledcAttachPin(8,0);
        // ledcAttachPin(7,1);
        // ledcAttachPin(22,2);
        // ledcAttachPin(19,3);
        // leftMotor = new Motor(0);
        // rightMotor = new Motor(1);
        // leftIRSensor = new IRSensor(ADC1_CHANNEL_6);
        // rightIRSensor = new IRSensor(ADC1_CHANNEL_7);
        // leftMotor->attachPins(pwmOut1, dirOut1);
        // rightMotor->attachPins(pwmOut2, dirOut2);
        // robot = new DriveMotors(leftMotor, rightMotor, leftIRSensor, rightIRSensor);
        //
        //
        // xTaskCreate(
        //     test_drive,
        //     "Testing Drive",
        //     4000,
        //     &robot,
        //     0,
        //     NULL);

        // pinMode(rotaryA, INPUT_PULLUP);
        // pinMode(rotaryB, INPUT_PULLUP);
        // attachInterrupt(rotaryA, encoderRead, CHANGE);

        // limit switches
        //setupLimitSwitches();

        // xTaskCreate(
        //     raise_carriage_task,  // Task function
        //     "Carriage up/down",   // Name
        //     4096,                 // Stack size
        //     NULL,                 // Parameters
        //     3,                    // Priority
        //     &raise_carriage_handle // Handle
        // );
        // xTaskCreate(
        //     test_raise_carriage_task,  // Task function
        //     "Test carriage",   // Name
        //     4096,                 // Stack size
        //     NULL,                 // Parameters
        //     3,                    // Priority
        //     &test_raise_carriage_handle // Handle
        // );
        // xTaskCreate(
        //     poll_switch_task,     // Task function
        //     "Poll switches",      // Name
        //     4096,                 // Stack size
        //     NULL,                 // Parameters
        //     4,                    // Priority
        //     &poll_switch_handle   // Handle
        // );
        // DRIVING
        // xTaskCreate(
        //     test_drive,
        //     "Drive",
        //     4096,
        //     robot,
        //     1,
        //     nullptr
        // );
        
        // xTaskCreate(
        //     test_servo,
        //     "Servoing",
        //     4096,
        //     SG90,
        //     1,
        //     nullptr
        // );
        // configIRSensors();
        // attachDriveMotorPins(true);
    }

}

void loop()
{
    // MG996R.rotateTo(90);
    // pickUpPet();
    // delay(3000);
    if (!run) {
        // Serial.println("Testing carriage");
        // moveCarriage(!carriageHigh);
        // Serial.print("Carriage position now ");
        // Serial.println(carriageHigh);

        // delay(1000);

        // Serial.println("Testing claw");
        // extendClaw(clawFullyRetracted);
        // Serial.print("Claw retracted? ");
        // Serial.println(clawFullyRetracted);
        // Serial.print("Claw extended? ");
        // Serial.println(clawFullyExtended);
        // delay(1000);
        // PUT TEST CODE HERE
        
        // drive(speed);
        // delay(1000/pwmFreq);
        // testServo.setAngle(180);
        // delay(100);
        // testServo.setAngle(90);
        // delay(100);
        // testServo.setAngle(0);
        // delay(100);
        

        // if (!rotationTested) {
        //     testRotation();
        //     rotationTested=true; 
        // }
        // MG996R.rotateTo(90);
        // Serial.print("A: ");
        // Serial.print(digitalRead(rotaryA));
        // Serial.print(" B ");
        // Serial.print(digitalRead(rotaryB));
        // Serial.print(" ISR: ");
        // Serial.print(isrTrigger);
        // Serial.print(" ");
        // Serial.println(rotaryPosition);
        // // robot.driveStraight(2000,1);
        // delay(400);

        //driving code
        // drive(speed, true);
        // delay(2);
        ////leftMotor->driveReverse(4095);
        // ledcWrite(rightPwmChannelFwd,100    );
        // ledcWrite(leftPwmChannelFwd,3000);
        // delay(1000);
        //robot.driveStraight(3000,1);

        ledcWrite(12,7864);
        delay(1000);
        Serial.println("At 270");
        ledcWrite(12,1638);
        delay(1000);
        Serial.println("At 0");
    }

//     // to be left empty, robot should run in the freeRTOS task scheduler
    }
    
//  }
// #include <Arduino.h>
// #include "driver/adc.h"

// void setup() {
//   Serial.begin(115200);
//   delay(1000);

//   adc1_config_width(ADC_WIDTH_12Bit); // 0–4095 resolution
//   adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_12); // GPIO32

//   Serial.println("Starting Hall Sensor detection...");
// }

// void loop() {
//   int raw = adc1_get_raw(ADC1_CHANNEL_4);  // Read ADC raw value

//   Serial.print("Hall Voltage: ");
//   Serial.print(raw);
 
//   // Magnet detection logic (outside 1600–1900 range = magnet detected)
//   if (raw > 1820 || raw < 1600) {
//     Serial.println(" | Magnet: Yes");
//   } else {
//     Serial.println(" | Magnet: No");
//   }

//   delay(500);  // Delay between readings
// }

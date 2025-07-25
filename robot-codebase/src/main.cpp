#include <Arduino.h>
#include "driver/ledc.h"
#include "driver/adc.h"
#include <ESP32Servo.h>
#include "driver/gpio.h"
#include <HardwareSerial.h>
#include "driver/pcnt.h"
#include "constants.h"
#include "hardware/CustomServo.h"
#include "components/DriveMotors.h"

// TRUE IF RUNNING, FALSE IF TESTING
bool run = false;
// global variables and task handles

TaskHandle_t drive_handle = nullptr;
TaskHandle_t grab_handle = nullptr;
TaskHandle_t reverse_handle = nullptr;
TaskHandle_t raise_basket_handle = nullptr;
TaskHandle_t home_handle = nullptr;
TaskHandle_t full_turn_handle = nullptr;
TaskHandle_t detect_handle = nullptr;
TaskHandle_t idle_handle = nullptr;

// initialize serial port for Pi communication

HardwareSerial Serial2Pi(0); // for UART 0

// robot properties/location
volatile int speed = defaultSpeed;    // average speed
volatile bool carriageHigh = false; // true if high, false if low
int petsPickedUp = 0;

// PID vars
int distance = 0; // right = positive
int last_distance = 0;
int p = 0;
int d = 0;
int m = 0; // counts cycles since the reading last changed; more cycles = smaller d
int q = 0; // equivalent to m_last (last previous value of m)
int ctrl = 0;
//int dir1 = 1;
//int dir2 = 1;
bool leftOnTape = 1;
bool rightOnTape = 1;
int leftVal = 0;
int rightVal = 0;
int leftSpeed = 0;
int rightSpeed = 0;
int lastOnTape = 0; // -1: left; 1: right

// pet location vars
double petX = 0;
double petY = 0;
double petW = 0;
double petH = 0;

//booleans for pick up
bool closeEnough = false;
bool clawCentered = false;
bool anglePastThreshold = false;

// other vars
unsigned long startTime = 0;
uint32_t reverseMultiplier = 0.3; // percentage speed of average speed for driving backwards
int rotaryCounter = 0;
int16_t rotaryMax = 0;
int16_t rotaryMin = 0;

// for limit switches
volatile bool carriageSwitchHit = false;
volatile bool clawFullyExtended = false;
volatile bool clawFullyRetracted = false;

// servos

// for closing the claw
CustomServo SG90(SG90Pin, clawClosingPWMChannel, clawOpenPos, servoFreq, servoMinDuty, servoMaxDuty);
// uint32_t SG90Pos = 0;

// for rotating turret
CustomServo MG996R(MG996RPin,carriageServoPWMChannel, carriageForwardPos, servoFreq, servoMinDuty, servoMaxDuty);
//uint32_t MG996RPos = 0;

//  motor declarations
Motor* leftMotor;
Motor* rightMotor;
IRSensor* leftIRSensor;
IRSensor* rightIRSensor;
DriveMotors* robot;

//Motor carriageMotor(carriageHeightPWMChannel);

// function declarations
void resetVars();
int distToTape();
double angleToCenter(double petX);
void driveMotor(int pwmCh, int dirPin, int speed, int dir);
void switchMotorDirection(int pwmCh, int dirPin, int speed, int newDir);
void drive(int avgSpeedInput);
void moveCarriage(bool up);
void pickUpPet();
void dropPetInBasket();
void home();
void rotaryEncoderSetup();

bool heightsForPickup[6] = {false, false, true, true, false, false}; //false = low, true = high
bool pickupSide[6] = {false, true, true, true, true, false}; // false = left, true = right
double petDistToTape[6] = {10.0, 14.0, 14.0, 14.0, 14.0, 14.0}; //distances in inches from tape

// TEST PARAMETERS


/**
 * resets pet detection related variables after pickup
 */
void resetVars() {
    MG996R.rotateTo(90);
    closeEnough = false;
    clawCentered = false;
    anglePastThreshold = false;
    speed=1900;
}

/**
 * distToTape - calculates the distance to the tape based on the IR sensor readings
 *
 * @return an integer indicating the robot's distance from the tape, either +/-5, +/-1, or 0 (on the line)
 */
int distToTape()
{
    int dist = 0;
    leftOnTape = adc1_get_raw(ADC1_CHANNEL_6) > thresholdL; // adc1 ch6 = pin 34
    rightOnTape = adc1_get_raw(ADC1_CHANNEL_7) > thresholdR; // adc1 ch7 = pin 35
    if (leftOnTape == 1 && rightOnTape == 1)
    {
        dist = 0;
    }
    else if (leftOnTape == 0 && rightOnTape == 1)
    {
        // only right sensor is on tape -> robot is to the left
        dist = -1;
        lastOnTape = 1;
    }
    else if (leftOnTape == 1 && rightOnTape == 0)
    {
        // only left sensor is on tape -> robot is to the left
        dist = 1;
        lastOnTape = -1;
    }
    else if (leftOnTape == 0 && rightOnTape == 0 && lastOnTape == 1)
    {
        // neither on tape but right was last one on tape
        dist = -5;
    }
    else
    {
        // neither on tape but left was last one on tape
        dist = 5;
    }
    return dist;
}

/**
 * calculates angle to center of pet. Note that the input image is flipped vertically.
 * @param pet_x_coord center of pet's x coordinate
 * @return angle between -31 (pet on very left of frame) to +31 (pet on very right of frame)
 */
double angleToCenter(double petX) {
    double result= (petX-(double)imgSize/2)*horizontal_fov;
    Serial2Pi.printf("Turret off by: %.2lf\n",result);
    return result;
}

/**
 * drives the robot forward with PID control
 * @param avgSpeedInput the average speed of the robot while driving
 */
void drive(int avgSpeedInput)
{
    last_distance = distance;
    distance = distToTape();

    if (last_distance != distance)
    {
        q = m;
        m = 1;
    }

    p = defaultKProp * distance;
    d = (int)((float)defaultKDeriv * (float)(distance - last_distance) / (float)(q + m));
    // i+=ki*distance;
    ctrl = (int)(p + d);
    m++;

    leftSpeed = constrain(avgSpeedInput - ctrl, minSpeed, maxSpeed);
    rightSpeed = constrain(avgSpeedInput + ctrl, minSpeed, maxSpeed);

    /* 
    // Greg bridge
    digitalWrite(dirOut1, dir1);
    digitalWrite(dirOut2, dir2); 
    ledcWrite(leftPwmChannel,leftSpeed);
    ledcWrite(rightPwmChannel,rightSpeed);
    */

    //Andre bridge
    ledcWrite(leftPwmChannelFwd, leftSpeed);
    ledcWrite(rightPwmChannelFwd, rightSpeed);
    ledcWrite(leftPwmChannelBwd, 0);
    ledcWrite(rightPwmChannelBwd, 0);

    leftVal = adc1_get_raw(ADC1_CHANNEL_6);
    rightVal = adc1_get_raw(ADC1_CHANNEL_7);

    Serial.print("Left reading:");
    Serial.print(leftVal);
    Serial.print("   Right reading:");
    Serial.println(rightVal);
    // Serial.print("Speed left:");
    // Serial.println(leftSpeed);
    // Serial.print("Speed right:");
    // Serial.println(rightSpeed);
}

void attachMotorPins(int pwmChannel, int pwmPin, int dirPin) {
    ledcSetup(pwmChannel,pwmFreq,12);
    ledcAttachPin(pwmPin,pwmChannel);
    pinMode(dirPin,OUTPUT);
}

void attachAndreMotorPins(int pwmCh1, int pwmCh2, int pwmPin1, int pwmPin2) {
    ledcSetup(pwmCh1, pwmFreq, 12);
    ledcSetup(pwmCh2, pwmFreq, 12);
    ledcAttachPin(pwmPin1, pwmCh1);
    ledcAttachPin(pwmPin2, pwmCh2);
}

void attachDriveMotorPins() {
    ledcSetup(leftPwmChannelFwd, pwmFreq, 12);
    ledcSetup(leftPwmChannelBwd, pwmFreq, 12);
    ledcAttachPin(pwmOut1, leftPwmChannelFwd);
    ledcAttachPin(dirOut1, leftPwmChannelBwd);

    ledcSetup(rightPwmChannelFwd, pwmFreq, 12);
    ledcSetup(rightPwmChannelBwd, pwmFreq, 12);
    ledcAttachPin(dirOut2, rightPwmChannelFwd);
    ledcAttachPin(pwmOut2, rightPwmChannelBwd);
}

void configIRSensors() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6,ADC_ATTEN_DB_12); // ir sensor inputs (pin 34/35)
    adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_DB_12);

}

void driveMotor(int pwmCh, int dirPin, int speed, int dir) {
    ledcWrite(pwmCh, speed);
    digitalWrite(dirPin,dir);
}

void driveAndreMotor(int pwmCh1, int pwmCh2, int speed, int dir) {
    if (dir==1) {
        ledcWrite(pwmCh1, speed);
    } else {
        ledcWrite(pwmCh2, speed);
    }
}

void stopMotor(int pwmCh) {
    ledcWrite(pwmCh,0);
}

void stopDrive() {
    stopMotor(leftPwmChannelFwd);
    stopMotor(leftPwmChannelBwd);
    stopMotor(rightPwmChannelFwd);
    stopMotor(rightPwmChannelFwd);
}

void switchMotorDirection(int pwmCh, int dirPin, int speed, int newDir) {
    stopMotor(pwmCh);
    delay(5);
    digitalWrite(dirPin, newDir);
    ledcWrite(pwmCh,speed);
}

/** 
 * changes carriage height
 * @param up, true if moving up and false if moving down
 */
void moveCarriage(bool up) {
    while(!carriageSwitchHit) {
        driveMotor(carriageHeightPWMChannel,carriageMotorDir,carriageSpeed,up);
        if (up) {
            Serial.println("Moving carriage upwards");
        } else {
            Serial.println("Moving carriage downwards");
        }
        delay(400);
    }
    if (carriageSwitchHit) {
        if (carriageHigh) {
            Serial.println("Carriage hit high switch");
        } else {
            Serial.println("Carriage hit low switch");
        }
        
    }
}

void extendClaw (bool outwards) {
    if (outwards) {
        Serial.println("Claw extending");
        while (!clawFullyExtended) {
            clawFullyRetracted = false;
            driveMotor(clawExtPWMChannel,clawExtMotorDir, clawExtSpeed, 1);
        }
    } else {
        Serial.println("Claw retracting");
        while (!clawFullyRetracted) {
            clawFullyExtended = false;
            driveMotor(clawExtPWMChannel,clawExtMotorDir, clawExtSpeed, 0);
        }
    }
}

void closeClaw(bool close) {
    if (close) {
        SG90.rotateTo(clawClosedPos);
        Serial.println("claw closing");
    } else {
        SG90.rotateTo(clawOpenPos);
        Serial.println("claw opening");
    }
}

/**
 * picks up pet
 */
void pickUpPet() {
    if (carriageHigh != heightsForPickup[petsPickedUp]) {
        moveCarriage(heightsForPickup[petsPickedUp]);
    }
    extendClaw(true);
    // receive input from hall effect
    closeClaw(true);
    delay(500);
    petsPickedUp++;
    Serial2Pi.printf("Pet picked up!\n");
    return dropPetInBasket(); // START DROP SEQUENCE
}

void dropPetInBasket() {
    if (!carriageHigh) {
        moveCarriage(true); // moves carriage up if it's low
    }
    
    //rotate to max angle
    if (MG996R.getPosition() > carriageForwardPos) {
        Serial.println("Rotating to right side to drop pet");
        MG996R.rotateTo(carriageMaxRightPos);
    } else {
        Serial.println("Rotating to left side to drop pet");
        MG996R.rotateTo(carriageMaxLeftPos);
    }

    extendClaw(false); // retract claw
    closeClaw(false); // open claw
    delay(1000); // give time to drop pet
    extendClaw(true); // re extend claw
    prepareForNextPickup();
}

void prepareForNextPickup() {
    if (pickupSide[petsPickedUp]) { //next pickup is at right side
        MG996R.rotateTo(carriageForwardPos+45);
    } else { // left side
        MG996R.rotateTo(carriageForwardPos-45);
    }

    moveCarriage(heightsForPickup[petsPickedUp]);
}

/**
 * Runs the homing sequence for the robot
 */
void home()
{
    /**
     * Code for homing sequence to run on startup, including:
     * Homing DC motors using limit switches (2 motors)
     * Setting all servo motor positions to 0
     */

    uint32_t switchHit;
    /*
    SG90Pos = 0;
    DSPos = 0;
    MG996RPos = 0;

    SG90.write(SG90Pos);
    DS.write(DSPos);
    MG996R.write(MG996RPos);
    */

    // find limits of the claw
    driveMotor(clawExtPWMChannel, clawExtMotorDir, homeSpeed, 0);
    xTaskNotifyWait(0, 0xFFFFFFFF, &switchHit, portMAX_DELAY);
    if (switchHit == 3)
    {
        pcnt_get_counter_value(PCNT_UNIT, &rotaryMin);
    }
    else if (switchHit == 4)
    {
        pcnt_get_counter_value(PCNT_UNIT, &rotaryMax);
    }

    driveMotor(clawExtPWMChannel, clawExtMotorDir, homeSpeed, 1);
    xTaskNotifyWait(0, 0xFFFFFFFF, &switchHit, portMAX_DELAY);
    if (switchHit == 3)
    {
        pcnt_get_counter_value(PCNT_UNIT, &rotaryMin);
    }
    else if (switchHit == 4)
    {
        pcnt_get_counter_value(PCNT_UNIT, &rotaryMax);
    }
    stopMotor(clawExtPWMChannel);

    driveMotor(carriageHeightPWMChannel, carriageMotorDir, homeSpeed, 0);
    xTaskNotifyWait(0, 0xFFFFFFFF, &switchHit, portMAX_DELAY);
    if (switchHit == 1)
    {
        stopMotor(carriageHeightPWMChannel);
    }
    else if (switchHit == 2)
    {
        driveMotor(carriageHeightPWMChannel, carriageMotorDir, homeSpeed, 1);
        xTaskNotifyWait(0, 0xFFFFFFFF, &switchHit, portMAX_DELAY);
        stopMotor(carriageHeightPWMChannel);
    }
}

/**
 * sets up the PCNT counter using the two rotaryEncoderPins, an overflow limit of 10000,
 * and a filter time of  1000 clock cycles.
 */
void rotaryEncoderSetup()
{
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = rotaryEncoderPinA, // Only pulse pin (A)
        .ctrl_gpio_num = rotaryEncoderPinB, // Added direction pin B
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC, // Count up on positive edge
        .neg_mode = PCNT_COUNT_DIS, // Ignore falling edge (or count if needed)
        .counter_h_lim = 1000, // High limit (for overflow check)
        .counter_l_lim = 0,      // Low limit (optional)
        .unit = PCNT_UNIT,
        .channel = PCNT_CHANNEL_0,
    };
    pinMode(extraGND,OUTPUT);
    digitalWrite(extraGND,LOW);

    pcnt_unit_config(&pcnt_config);

    // Optional: filter out noise shorter than 1000 clock cycles
    pcnt_set_filter_value(PCNT_UNIT, 1000);
    pcnt_filter_enable(PCNT_UNIT);

    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);
    pcnt_counter_resume(PCNT_UNIT);
}

// Another ISR implementation for the start button to go (can also be a switch)
void IRAM_ATTR startButtonPressedISR()
{
    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(idle_handle, &hpw);
    portYIELD_FROM_ISR(&hpw);
}

void IRAM_ATTR carriageLowPressedISR()
{
    carriageHigh = false;
    carriageSwitchHit = true;
    // BaseType_t hpw = pdFALSE;
    // xTaskNotifyFromISR(home_handle, 0x01, eSetBits, &hpw);
    // portYIELD_FROM_ISR(&hpw);
}

void IRAM_ATTR carriageHighPressedISR()
{
    carriageHigh = true;
    carriageSwitchHit = true;
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

// freeRTOS tasks

/**
 * this task operates the main driving system of the robot, including PID control. It also includes a hard-coded stop c
 * condition if the time reaches 90 seconds, at which point it signals the full_turn task to execute at max priority.
 * @param parameters no parameters for this task
 */
void drive_task(void *parameters)
{

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // this creates an infinite loop, but it will be interrupted by other actions
    for (;;)
    {

        drive(speed);

        if (millis() - startTime > 90000)
        {
            xTaskNotifyGive(&full_turn_handle);
        }
        // the vTaskDelay function takes in ticks as a time measurement, so / portTick_PERIOD_MS converts to ms
        vTaskDelay(4 / portTICK_PERIOD_MS);
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
                int currentAngle=MG996R.getPosition();
                Serial2Pi.printf("servo angle: %d\n",currentAngle);

                //check if pet big enough for pickup
                closeEnough = petArea > areaThresholdForPickup;           

                // check if angle is correct (off forward direction by at least 75 deg)
                anglePastThreshold = (currentAngle < carriageForwardPos - angleThreshold ||
                                  currentAngle > carriageForwardPos + angleThreshold);

                //rotate turret
                double rotate_const=1.0;
                if (abs((int)petX-imgSize/2) > clawCenterThreshold) { //tolerance of 20; center of pet in pixels 140-180 is good enough
                    MG996R.rotateBy((int)(angleToCenter(petX)*rotate_const));
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
                    stopDrive();
                    vTaskSuspendAll();
                    pickUpPet();      
                    xTaskResumeAll();   
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


// add more functions here

void setup()
{
    // put your setup code here, to run once:
    if (run) {
        // initialize UART connection to the Pi
        Serial2Pi.begin(115200, SERIAL_8N1, RXPin, TXPin);
        Serial2Pi.write("Hello from the ESP32!");

        // initialize basic pin connections

        // needs to be pull up for encoder to function properly (I think <-- TO BE TESTED)
        pinMode(rotaryEncoderPinA, INPUT_PULLUP);
        pinMode(rotaryEncoderPinB, INPUT_PULLUP);

        // initialize adc channels for pwm signals to operate drive
        adc1_config_width(ADC_WIDTH_BIT_12);

        adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_12); // ir sensor inputs (pin 32)
        adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_12); // pin 33
        adc2_config_channel_atten(ADC2_CHANNEL_7, ADC_ATTEN_DB_12); // pin 27 = p_pot (to be removed later)
        adc2_config_channel_atten(ADC2_CHANNEL_6, ADC_ATTEN_DB_12); // pin 14 = d_pot

        ledcSetup(leftPwmChannelFwd, 250, 12); // middle number: duty cycle resolution in hz
        ledcSetup(rightPwmChannelFwd, 250, 12);
        ledcAttachPin(pwmOut1, leftPwmChannelFwd);
        ledcAttachPin(pwmOut2, rightPwmChannelFwd); // both motors controlled by same pwm channel

        ledcSetup(carriageHeightPWMChannel, 250, 12);
        ledcAttachPin(carriageMotorPWM, carriageHeightPWMChannel);

        ledcSetup(clawExtPWMChannel, 250, 12);
        ledcAttachPin(clawExtMotorPWM, clawExtPWMChannel);

        // attach pins for ISRs

        pinMode(startSwitch, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(startSwitch), startButtonPressedISR, RISING);

        // starts the PCNT setup code
        rotaryEncoderSetup();
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
    }

    if (!run) {
        Serial.begin(9600);
        Serial.println("Serial starting");
        rotaryEncoderSetup();

        // carriage mvt limit switches
        // pinMode(carriageLOW, INPUT_PULLUP);
        // pinMode(carriageHIGH, INPUT_PULLUP);
        // attachInterrupt(digitalPinToInterrupt(carriageLOW), carriageLowPressedISR, RISING); 
        // attachInterrupt(digitalPinToInterrupt(carriageHIGH), carriageHighPressedISR, RISING); 

        // claw extension limit switches
        pinMode(clawExtendedSwitch, INPUT_PULLUP);
        pinMode(clawRetractedSwitch, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(clawExtendedSwitch), clawFullExtensionPressedISR, RISING); 
        attachInterrupt(digitalPinToInterrupt(clawRetractedSwitch), clawFullRetractionPressedISR, RISING);


        // DRIVING
        // configIRSensors();
        // attachDriveMotorPins();
    }
}

void loop()
{
    /*
    //carriage mvt
    moveCarriage(!carriageHigh);
    carriageSwitchHit=false;
    Serial.println("resetting");
    delay(200);
    */

    //rotary encoder
    // int16_t count = 0;
    // pcnt_get_counter_value(PCNT_UNIT, &count);
    // Serial.print("Encoder Count: ");
    // Serial.println(count);
    // delay(300);

    
    if (!run) {
        // PUT TEST CODE HERE

        //driving code
        drive(speed);
        delay(2);
    }

    // to be left empty, robot should run in the freeRTOS task scheduler
}

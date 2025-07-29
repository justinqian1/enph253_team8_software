#include <Arduino.h>
#include "driver/ledc.h"
#include "driver/adc.h"
#include <ESP32Servo.h>
#include "driver/gpio.h"
#include <HardwareSerial.h>
#include "driver/pcnt.h"
#include "constants.h"
#include "hardware/CustomServo.h"
#include "components/RobotWheels.h"

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
volatile int speed = 1900;    // average speed
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
float petX = 0;
float petY = 0;
float petW = 0;
float petH = 0;

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
volatile int rotaryPosition = 0;
volatile int lastEncodedBitValue  = 0;
constexpr int lookupTable[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
volatile int isrTrigger = 0;
volatile unsigned long lastTime = 0;
// servos

// for closing the claw
Servo SG90;
uint32_t SG90Pos = 0;

// for lifting the basket
Servo DS;
uint32_t DSPos = 0;

// for rotating turret
CustomServo MG996R(MG996RPin,carriageServoPWMChannel, 90, 50, 500, 2500);
//uint32_t MG996RPos = 0;

//  motor declarations
Motor leftMotor(0, 8, 1, 7);
Motor rightMotor(2, 19, 3, 22);
IRSensor leftIRSensor(ADC1_CHANNEL_0);
IRSensor rightIRSensor(ADC1_CHANNEL_1);
RobotWheels robot(leftMotor, rightMotor, leftIRSensor, rightIRSensor);

//Motor carriageMotor(carriageHeightPWMChannel);

// function declarations
int distToTape();
float angleToCenter(float pet_x_coord);
void driveMotor(int motorPWM, int directionPin, int speed, int direction);
void drive(int avgSpeedInput);
void stopMotor(int motorPWM);
void driveReverse(int avgSpeedInput);
void stopDrive();
void stopAllMotors();
void rotateTurretBy(int pos);
void rotateTurretTo(int pos);
void pickUpPet();
void dropPetInBasket();
void home();
void PCNTsetup();

bool heightsForPickup[6] = {false, false, true, true, false, false}; //false = low, true = high

// TEST PARAMETERS



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
float angleToCenter(float pet_x_coord) {
    return (pet_x_coord-(float)imgSize/2)*horizontal_fov;
}

/**
 * drives the robot forward with PID control
 * @param avgSpeedInput the average speed of the robot while driving
 */
void drive(int avgSpeedInput)
{
    /*
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

    digitalWrite(dirOut1, dir1);
    digitalWrite(dirOut2, dir2);
    leftSpeed = max(avgSpeedInput - ctrl, minSpeed);
    leftSpeed = min(leftSpeed, maxSpeed);
    rightSpeed = max(avgSpeedInput + ctrl, minSpeed);
    rightSpeed = min(rightSpeed, maxSpeed);
    ledcWrite(leftPwmChannel,leftSpeed);
    ledcWrite(rightPwmChannel,rightSpeed);
    //driveMotor(leftPwmChannel, dirOut1, leftSpeed, 1);
    //driveMotor(rightPwmChannel, dirOut2, rightSpeed, 1);
    leftVal = adc1_get_raw(ADC1_CHANNEL_6);
    rightVal = adc1_get_raw(ADC1_CHANNEL_7);

    Serial.print("Left reading:");
    Serial.println(leftVal);
    Serial.print("Right reading:");
    Serial.println(rightVal);
    Serial.print("Speed left:");
    Serial.println(leftSpeed);
    Serial.print("Speed right:");
    Serial.println(rightSpeed);
    */
}

/**
 * driveReverse - drives the robot in reverse in a straight line without PID control
 *
 * @avgSpeedInput the speed at which to drive backwards
 */
void driveReverse(const int avgSpeedInput)
{
    driveMotor(leftPwmChannel, dirOut1, avgSpeedInput, 0);
    driveMotor(rightPwmChannel, dirOut2, avgSpeedInput, 0);
}

/**
 * stopDrive - stops both motors entirely
 *              no guarantees about stopping motion of robot entirely (backlash, momentum, etc...)
 */
void stopDrive()
{
    ledcWrite(leftPwmChannel, 0);
    ledcWrite(rightPwmChannel, 0);
}

/**
 * Drives a specific motor and a given speed in a given direction, as opposed to drive which drives both wheel motors
 * @param motorPWM the PWM channel for the motor
 * @param directionPin the direction pin for the motor
 * @param direction the direction to drive, with 1 being right and 0 being left
 * @param speed the speed to drive the motor, in range 0 - 4095
 */
void driveMotor(const int motorPWM, const int directionPin, const int speed, const int direction)
{
    digitalWrite(directionPin, direction);
    ledcWrite(motorPWM, speed);
}

/**
 * Stops a specific motor
 * @param motorPWM the PWM channel for the motor
 */
void stopMotor(const int motorPWM)
{
    ledcWrite(motorPWM, 0);
}

/**
 * Stops all motors (generally for testing purposes)
 */
void stopAllMotors()
{
    ledcWrite(leftPwmChannel, 0);
    ledcWrite(rightPwmChannel, 0);
    ledcWrite(clawExtPWMChannel, 0);
    ledcWrite(carriageHeightPWMChannel, 0);
}

/** 
 * changes carriage height
 * @param up, true if moving up and false if moving down
 */
void moveCarriage(bool up) {
    while(!carriageSwitchHit) {
        //carriageMotor.driveMotor(carriageSpeed,up);
    }
}

/**
 * picks up pet
 */
void pickUpPet() {
    if (carriageHigh != heightsForPickup[petsPickedUp]) {
        moveCarriage(heightsForPickup[petsPickedUp]);
    }
    /*
    extend claw
    receive input from hall effect
    close claw
    */
    sleep(1);
    petsPickedUp++;
    Serial2Pi.printf("Pet picked up!\n");
    return dropPetInBasket(); // START DROP SEQUENCE
}

void dropPetInBasket() {
    if (!carriageHigh) {
        moveCarriage(true); // moves carriage up if it's low
    }
    /*
    rotate turret
    retract claw
    open claw
    pause
    extend claw
    rotate claw/lower claw depending on next pickup location
    */
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
// OBSOLETE - NO LONGER USING PCNT
/**
 * sets up the PCNT counter using the two rotaryEncoderPins, an overflow limit of 10000,
 * and a filter time of  1000 clock cycles.
 */
void PCNTSetup()
{
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = rotaryA, // Only pulse pin (A)
        .ctrl_gpio_num = rotaryB, // Added direction pin B
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC, // Count up on positive edge
        .neg_mode = PCNT_COUNT_DIS, // Ignore falling edge (or count if needed)
        .counter_h_lim = 100, // High limit (for overflow check)
        .counter_l_lim = -100,      // Low limit (optional)
        .unit = PCNT_UNIT,
        .channel = PCNT_CHANNEL_0,
    };

    pcnt_unit_config(&pcnt_config);

    // Optional: filter out noise shorter than 1000 clock cycles
    pcnt_set_filter_value(PCNT_UNIT, 100);
    pcnt_filter_enable(PCNT_UNIT);

    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);
    pcnt_counter_resume(PCNT_UNIT);
}

// This is an ISR implementation of the button press interrupt for the reversing and basket raising mechanism
// it sends a notification to the reverse_task to commence (after three presses)
void IRAM_ATTR basketSwitchPressedISR()
{
    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(reverse_handle, &hpw);
    portYIELD_FROM_ISR(&hpw);
}

// Another ISR implementation for the start button to go (can also be a switch)
void IRAM_ATTR startButtonPressedISR()
{
    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(idle_handle, &hpw);
    portYIELD_FROM_ISR(&hpw);
}

void IRAM_ATTR vertClawLowPressedISR()
{
    carriageSwitchHit = true;
    carriageHigh = false;
    BaseType_t hpw = pdFALSE;
    xTaskNotifyFromISR(home_handle, 0x01, eSetBits, &hpw);
    portYIELD_FROM_ISR(&hpw);
}

void IRAM_ATTR vertClawHighPressedISR()
{
    carriageSwitchHit = true;
    carriageHigh = true;
    BaseType_t hpw = pdFALSE;
    xTaskNotifyFromISR(home_handle, 0x02, eSetBits, &hpw);
    portYIELD_FROM_ISR(&hpw);
}

void IRAM_ATTR horiClawLowPressedISR()
{
    BaseType_t hpw = pdFALSE;
    xTaskNotifyFromISR(home_handle, 0x03, eSetBits, &hpw);
    portYIELD_FROM_ISR(&hpw);
}

void IRAM_ATTR horiClawHHighPressedISR()
{
    BaseType_t hpw = pdFALSE;
    xTaskNotifyFromISR(home_handle, 0x04, eSetBits, &hpw);
    portYIELD_FROM_ISR(&hpw);
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

// for communication with Pi, I was thinking the detect and grab tasks would bounce between each other
/**
 * this task operates the grabbing system of the robot
 * @param parameters no parameters for this task
 */
void grab_task(void *parameters)
{

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // grabbing code here, feel free to change parameters
}

/**
 * this task operates the reverse driving of the robot, limited to straight-line motion. This task also covers alignment
 * with the zipline by reversing and then sending a signal to the raise_basket task.
 * @param parameters no parameters for this task
 */
void reverse_task(void *parameters)
{

    // waits for third switch press before intiating reverse + basket raise (first hit at the door, second when going up the ramp, final on the zipline)
    uint32_t pressCount = 0;
    while (pressCount < 3)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        pressCount++;
    }

    for (;;)
    {
        // reverse code for aligning with zipline
        vTaskSuspend(&drive_handle);
        // reverse direction and drive slowly backwards (depending on reverseMultiplier) for two seconds
        driveReverse(speed * reverseMultiplier);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        // stop motors for basket raise
        stopDrive();

        // signals to start basket raising code
        xTaskNotifyGive(&raise_basket_handle);
    }
}

/**
 * this task operates the raising of the basket for the zipline. It suspends the drive task temporarily while raising
 * the basket.
 * @param parameters no parameters for this task
 */
void raise_basket_task(void *parameters)
{

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    stopDrive();
    // raise basket by rotating SG90 by 90 degrees slowly (hence the delays)
    // there used to be a for(;;) loop here but I didn't really see why there was one so I removed it.
    while (DSPos < 90)
    {
        MG996R.write(++DSPos);
        vTaskDelay(10 / portTICK_PERIOD_MS);
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
 * this task operates a full 180 degree turn of the robot, ideally without losing the line. It is a safeguard for the
 * case in which the 90-second limit is reached. It suspends all other tasks during its operation, and only resumes
 * the drive task when completed.
 * @param parameters no parameters for this task
 */
void full_turn_task(void *parameters)
{
    // full 180 turn sequence here, for once 90 second hard limit reached
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // abort all other actions
    vTaskSuspendAll();

    // hard code in 90-degree turn

    // drive back to the start  (hopefully 180 turn lines up with tape)
    vTaskResume(&drive_handle);
}

/**
 * this task handles detection of pets through serial communication with the Raspberry Pi 5. It signals the home_claw_task
 * to update its position. It also signals when a pet is close enough to be picked up to operate the grab task.
 * @param parameters no parameters for this task
 */
void detect_task(void *parameters)
{
    /*
    // detection code for determining pet location
    while (1) {
        if (Serial2Pi.available()) {
            String line = Serial2Pi.readStringUntil('\n');
            if (line.length()==1) {
                // no pets; continue as normal
                //Serial.printf("no pets\n");
            } else {
                // pet in visual range
                float rotate_const=0.5;
                sscanf(line.c_str(), "%f,%f,%f,%f", &petX, &petY, &petW, &petH);

                //rotate turret
                if (abs(petX-img_size/2) > 20) { //tolerance of 20; center of pet in pixels 140-180 is good enough
                    rotateTurret((int)angleToCenter(petX)*rotate_const);
                }

                // slow down robot/initiate pick up sequence
                float petArea = petW*petH;
                if (petArea > 1500) { //within ~2ft of pet
                    int currentAngle = MG996R.read();
                    if (currentAngle < angleForward - angleThreshold || currentAngle > angleForward + angleThreshold) {
                        // arm is at nearly 90 degree angle -> initiate pick up
                        vTaskSuspendAll();
                        pickUpPet();      
                        xTaskResumeAll();                  
                    } else {
                        // not close enough - update speed
                        int tempSpeedCeiling = (int)(petArea*-0.2)+1900; // arbitrary function for now
                        int currentSpeed = speed;
                        tempSpeedCeiling=max(tempSpeedCeiling,0); // make sure it's positive
                        speed=min((int)speed,tempSpeedCeiling);
                        speed=min(currentSpeed,tempSpeedCeiling);
                    }
                }
                //Serial.printf("x=%.2f y=%.2f w=%.2f h=%.2f\n", pet_x, pet_y, pet_w, pet_h);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
        */
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

void test_drive(void *parameters) {
    // DriveMotors* robot = static_cast<DriveMotors*>(parameters);
    // for (;;) {
    //     Serial.println("before switch");
    //     robot->driveLeftMotor(speed,HIGH);
    //     Serial.println(leftMotor->currentDirection);
    //     vTaskDelay(1000);
    //     Serial.println("after switch");
    //     robot->driveRightMotor(speed,LOW);
    //     vTaskDelay((1000/pwmFreq) / portTICK_PERIOD_MS);
    // }
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

        ledcSetup(leftPwmChannel, 250, 12); // middle number: duty cycle resolution in hz
        ledcSetup(rightPwmChannel, 250, 12);
        ledcAttachPin(pwmOut1, leftPwmChannel);
        ledcAttachPin(pwmOut2, rightPwmChannel); // both motors controlled by same pwm channel

        ledcSetup(carriageHeightPWMChannel, 250, 12);
        ledcAttachPin(carriageMotorPWM, carriageHeightPWMChannel);

        ledcSetup(clawExtPWMChannel, 250, 12);
        ledcAttachPin(clawExtMotorPWM, clawExtPWMChannel);

        // attach pins for ISRs

        pinMode(startSwitch, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(startSwitch), startButtonPressedISR, RISING);
        pinMode(basketSwitch, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(basketSwitch), basketSwitchPressedISR, RISING);

        // starts the PCNT setup code
        PCNTSetup();
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

        xTaskCreate(
            grab_task,   // function to be run
            "Grabbing",  // description of task
            1000,        // bytes allocated to this stack
            NULL,        // parameters, dependent on function
            2,           // priority
            &grab_handle // task handle
        );

        xTaskCreate(
            reverse_task,   // function to be run
            "Reversing",    // description of task
            1000,           // bytes allocated to this stack
            NULL,           // parameters, dependent on function
            3,              // priority
            &reverse_handle // task handle
        );

        xTaskCreate(
            raise_basket_task,   // function to be run
            "Raising Basket",    // description of task
            1000,                // bytes allocated to this stack
            NULL,                // parameters, dependent on function
            3,                   // priority
            &raise_basket_handle // task handle
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

        // high priority task since it overrides all other functions once 90 seconds are triggered
        xTaskCreate(
            full_turn_task,  // function to be run
            "Turning (360)", // description of task
            1000,            // bytes allocated to this stack
            NULL,            // parameters, dependent on function
            5,               // priority
            &detect_handle   // task handle
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

        // Servo setups
        SG90.setPeriodHertz(50);
        SG90.attach(SG90Pin, 500, 2400);

        DS.setPeriodHertz(50);
        DS.attach(DSPin, 500, 2400);
    }

    if (!run) {
        Serial.begin(9600);
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
        /*
        CARRIAGE MVT TESTING
        carriageMotor.attachPins(carriageMotorPWM,carriageMotorDir);
        pinMode(vertClawLOW, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(vertClawLOW), vertClawLowPressedISR, CHANGE); // chat suggests "FALLING"
        */

        /*
        <old driving code>
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(ADC1_CHANNEL_6,ADC_ATTEN_DB_12); // ir sensor inputs (pin 34/35)
        adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_DB_12);

        ledcSetup(leftPwmChannel,pwmFreq,12);
        ledcSetup(rightPwmChannel,pwmFreq,12);
        ledcAttachPin(pwmOut1,leftPwmChannel);
        ledcAttachPin(pwmOut2,rightPwmChannel);
        pinMode(dirOut1,OUTPUT);
        pinMode(dirOut2,OUTPUT);
        // */
        // pinMode(rotaryA, INPUT_PULLUP);
        // pinMode(rotaryB, INPUT_PULLUP);
        // attachInterrupt(rotaryA, encoderRead, CHANGE);
        //PCNTSetup();
    }
        pinMode(rotaryA, INPUT_PULLUP);
        pinMode(rotaryB, INPUT_PULLUP);
        attachInterrupt(rotaryA, encoderRead, CHANGE);

}

void loop()
{
    if (!run) {
        // PUT TEST CODE HERE
        /*
        drive(speed);
        delay(1000/pwmFreq);
        testServo.setAngle(180);
        delay(100);
        testServo.setAngle(90);
        delay(100);
        testServo.setAngle(0);
        delay(100);
        */
         Serial.print("A: ");
        Serial.print(digitalRead(rotaryA));
        Serial.print(" B ");
        Serial.print(digitalRead(rotaryB));
        Serial.print(" ISR: ");
        Serial.print(isrTrigger);
        Serial.print(" ");
       Serial.println(rotaryPosition);
        // robot.driveStraight(2000,1);
         delay(400);
    }

    // to be left empty, robot should run in the freeRTOS task scheduler
}

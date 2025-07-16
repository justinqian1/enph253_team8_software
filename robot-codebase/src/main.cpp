#include <Arduino.h>
#include "driver/ledc.h"
#include "driver/adc.h"
#include <ESP32Servo.h>
#include "driver/gpio.h"
#include <HardwareSerial.h>
#include "driver/pcnt.h"

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

HardwareSerial Serial2Pi(2); // for UART 2

// PWM Channels

constexpr int leftPwmChannel = 0;
constexpr int rightPwmChannel = 1;
constexpr int carriagePWMChannel = 2;
constexpr int clawExtPWMChannel = 3;

// ESP32 pins
constexpr int pwmOut1 = 20; // outputs the pwm channel according to ledcAttachPin
constexpr int dirOut1 = 21;
constexpr int pwmOut2 = 22;
constexpr int dirOut2 = 19;
constexpr int irSensorLeft = 34;
constexpr int irSensorRight = 35;
constexpr int SG90Pin = 14;
constexpr int DSPin = 12;
constexpr int MG996RPin = 13;
constexpr int basketSwitch = 25;
constexpr int RXPin = 9; // I'm moving some pins around just for code simplicity but these can change later <-- NEED TO BE CHANGED, NOT IDEAL FOR UART
constexpr int TXPin = 10; // same as above
constexpr int startSwitch = 39;
constexpr int vertClawLOW = 26;
constexpr int vertClawHIGH = 32;
constexpr int horiClawLOW = 33;
constexpr int horiClawHIGH = 27;
constexpr int carriageMotorPWM = 1;
constexpr int carriageMotorDir = 3;
constexpr int clawExtMotorPWM = 8;
constexpr int clawExtMotorDir = 7;
constexpr int rotaryEncoderPinA = 4;
constexpr int rotaryEncoderPinB = 15;
// other pins: 27 = p_pot, 14 = d_pot

// constants
constexpr int thresholdL = 1800;
constexpr int thresholdR = 1800;
constexpr int maxSpeed = 4000; // set a max pwm output
constexpr int minSpeed = 0;    // set a min pwm output
constexpr int speed = 1600;    // set an average speed
constexpr int homeSpeed = 600; // set a motor speed for the homing sequence

// camera params
constexpr int img_size = 320;
constexpr float horizontal_fov = 62.2; //degrees

    // misc consexpr
    constexpr pcnt_unit_t PCNT_UNIT = PCNT_UNIT_0;

// PID vars
int distance = 0; // right = positive
int last_distance = 0;
int kp = 0;
int kd = 0;
int p = 0;
int d = 0;
int m = 0; // counts cycles since the reading last changed; more cycles = smaller d
int q = 0; // equivalent to m_last (last previous value of m)
int ctrl = 0;
int dir1 = 1;
int dir2 = 1;
bool leftOnTape = 1;
bool rightOnTape = 1;
int leftVal = 0;
int rightVal = 0;
int leftSpeed = 0;
int rightSpeed = 0;
int lastOnTape = 0; // -1: left; 1: right

// pet location vars
bool petDetected = 0;
float pet_x = 0;
float pet_y = 0;
float pet_w = 0;
float pet_h = 0;

// other vars
unsigned long startTime = 0;
uint32_t reverseMultiplier = 0.3; // percentage speed of average speed for driving backwards
int rotaryCounter = 0;
int16_t rotaryMax = 0;
int16_t rotaryMin = 0;

// servos
Servo SG90;
uint32_t SG90Pos = 0;

Servo DS;
uint32_t DSPos = 0;

Servo MG996R;
uint32_t MG996RPos = 0;

// function declarations
int distToTape();
void driveMotor(int motorPWM, int directionPin, int speed, int direction);
void drive(int avgSpeedInput);
void stopMotor(int motorPWM);
void driveReverse(int avgSpeedInput);
void stopDrive();
void stopAllMotors();
void rotateClaw();
void home();
void PCNTsetup();

/**
 * distToTape - calculates the distance to the tape based on the IR sensor readings
 *
 * @return an integer indicating the robot's distance from the tape, either +/-5, +/-1, or 0 (on the line)
 */
int distToTape()
{
    int dist = 0;
    leftOnTape = adc1_get_raw(ADC1_CHANNEL_4) > thresholdL;
    rightOnTape = adc1_get_raw(ADC1_CHANNEL_5) > thresholdR;
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
 * drives the robot forward with PID control
 * @param avgSpeedInput the average speed of the robot while driving
 */
void drive(const int avgSpeedInput)
{
    last_distance = distance;
    distance = distToTape();

    if (last_distance != distance)
    {
        q = m;
        m = 1;
    }

    adc2_get_raw(ADC2_CHANNEL_7, ADC_WIDTH_10Bit, &kp); // WILL BE REMOVED
    adc2_get_raw(ADC2_CHANNEL_6, ADC_WIDTH_10Bit, &kd);
    p = kp * distance;
    d = (int)((float)kd * (float)(distance - last_distance) / (float)(q + m));
    // i+=ki*distance;
    ctrl = (int)(p + d);
    m++;

    digitalWrite(dirOut1, dir1);
    digitalWrite(dirOut2, dir2);
    leftSpeed = max(avgSpeedInput - ctrl, minSpeed);
    leftSpeed = min(leftSpeed, maxSpeed);
    rightSpeed = max(avgSpeedInput + ctrl, minSpeed);
    rightSpeed = min(rightSpeed, maxSpeed);
    driveMotor(leftPwmChannel, dirOut1, leftSpeed, 1);
    driveMotor(rightPwmChannel, dirOut2, rightSpeed, 1);
    leftVal = adc1_get_raw(ADC1_CHANNEL_4);
    rightVal = adc1_get_raw(ADC1_CHANNEL_5);

    // Serial.print("Left reading:");
    // Serial.println(leftVal);
    // Serial.print("Right reading:");
    // Serial.println(rightVal);
    // Serial.print("Distance:");
    // Serial.println(distance);
    // Serial.print("kp:");
    // Serial.println(kp);
    // Serial.print("kd:");
    // Serial.println(kd);
    // Serial.print("Speed left:");
    // Serial.println(leftSpeed);
    // Serial.print("Speed right:");
    // Serial.println(rightSpeed);
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
    ledcWrite(carriagePWMChannel, 0);
}

void rotateClaw()
{
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
    SG90Pos = 0;
    DSPos = 0;
    MG996RPos = 0;

    SG90.write(SG90Pos);
    DS.write(DSPos);
    MG996R.write(MG996RPos);

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

    driveMotor(carriagePWMChannel, carriageMotorDir, homeSpeed, 0);
    xTaskNotifyWait(0, 0xFFFFFFFF, &switchHit, portMAX_DELAY);
    if (switchHit == 1)
    {
        stopMotor(carriagePWMChannel);
    }
    else if (switchHit == 2)
    {
        driveMotor(carriagePWMChannel, carriageMotorDir, homeSpeed, 1);
        xTaskNotifyWait(0, 0xFFFFFFFF, &switchHit, portMAX_DELAY);
        stopMotor(carriagePWMChannel);
    }
}

void PCNTSetup()
{
    pcnt_config_t pcnt_config = {
        .unit = PCNT_UNIT,
        .pulse_gpio_num = rotaryEncoderPin, // Only pulse pin (A)
        .ctrl_gpio_num = PCNT_PIN_NOT_USED, // No direction pin
        .channel = PCNT_CHANNEL_0,
        .pos_mode = PCNT_COUNT_INC, // Count up on positive edge
        .neg_mode = PCNT_COUNT_DIS, // Ignore falling edge (or count if needed)
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_KEEP,
        .counter_h_lim = 10000, // High limit (for overflow check)
        .counter_l_lim = 0      // Low limit (optional)
    };

    pcnt_unit_config(&pcnt_config);

    // Optional: filter out noise shorter than 1000 clock cycles
    pcnt_set_filter_value(PCNT_UNIT, 1000);
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
    BaseType_t hpw = pdFALSE;
    xTaskNotifyFromISR(home_handle, 0x01, eSetBits, &hpw);
    portYIELD_FROM_ISR(&hpw);
}

void IRAM_ATTR vertClawHighPressedISR()
{
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
    while (MG996RPos < 90)
    {
        MG996R.write(++MG996RPos);
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
    // detection code for determining pet location
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
    Serial.begin(115200);
    // initialize UART connection to the Pi
    Serial2Pi.begin(9600, SERIAL_8N1, RXPin, TXPin);
    Serial2Pi.write("Hello from the ESP32!");

    // initialize basic pin connections

    // needs to be pull up for encoder to function properly
    pinMode(rotaryEncoderPin, INPUT_PULLUP);

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

    ledcSetup(carriagePWMChannel, 250, 12);
    ledcAttachPin(carriageMotorPWM, carriagePWMChannel);

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
        6,           // priority
        &idle_handle // task handle
    );

    // Servo setups
    SG90.setPeriodHertz(50);
    SG90.attach(SG90Pin, 500, 2400);

    DS.setPeriodHertz(50);
    DS.attach(DSPin, 500, 2400);

    MG996R.setPeriodHertz(50);
    MG996R.attach(MG996RPin, 500, 2400);
}

void loop()
{
    // put your main code here, to run repeatedly:

    // to be left empty, robot should run in the freeRTOS task scheduler
}

#include "globals.h"

Motor* rightMotor =  nullptr;
Motor* leftMotor = nullptr;
IRSensor* leftIRSensor = nullptr;
IRSensor* rightIRSensor =  nullptr;
RobotWheels* robot = nullptr;
CustomServo* SG90 = nullptr;
CustomServo* MG996R = nullptr;
Motor* carriageMotor = nullptr;
Motor* clawExtMotor = nullptr;

TaskHandle_t drive_handle = nullptr;
TaskHandle_t grab_handle = nullptr;
TaskHandle_t home_handle = nullptr;
TaskHandle_t raise_carriage_handle = nullptr;
TaskHandle_t test_raise_carriage_handle = nullptr;
TaskHandle_t poll_switch_handle = nullptr;
TaskHandle_t full_turn_handle = nullptr;
TaskHandle_t detect_handle = nullptr;
TaskHandle_t idle_handle = nullptr;

volatile bool carriageHigh = false;
volatile bool carriageLow = false;
volatile bool clawFullyExtended = false;
volatile bool clawFullyRetracted = false;

// pet location vars
double petX = 0;
double petY = 0;
double petW = 0;
double petH = 0;

//booleans for pick up
bool closeEnough = false;
bool clawCentered = false;
bool anglePastThreshold = false;

// properties
volatile int speed = defaultSpeed;    // average speed
int petsPickedUp = 0;
bool rotationTested=false; // for testing
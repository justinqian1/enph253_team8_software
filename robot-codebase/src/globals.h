#include "hardware/CustomServo.h"
#include "hardware/HallSensor.h"
#include "hardware/IRSensor.h"
#include "hardware/Motor.h"
#include "hardware/RotaryEncoder.h"
#include "components/Claw.h"
#include "components/RobotWheels.h" 

// object pointers 

extern Motor* rightMotor;//(rightPwmChannelFwd, 20, rightPwmChannelBwd, 21);
extern Motor* leftMotor;//(leftPwmChannelFwd, 8, leftPwmChannelBwd, 7);
extern Motor* carriageMotor;
extern Motor* clawExtMotor;
extern IRSensor* leftIRSensor;//(ADC1_CHANNEL_6);
extern IRSensor* rightIRSensor;//(ADC1_CHANNEL_7);
extern RobotWheels* robot;//(leftMotor, rightMotor, leftIRSensor, rightIRSensor);
extern CustomServo* SG90;
extern CustomServo* MG996R;



extern TaskHandle_t drive_handle;
extern TaskHandle_t grab_handle;
extern TaskHandle_t home_handle;
extern TaskHandle_t raise_carriage_handle;
extern TaskHandle_t test_raise_carriage_handle;
extern TaskHandle_t poll_switch_handle;
extern TaskHandle_t full_turn_handle;
extern TaskHandle_t detect_handle;
extern TaskHandle_t idle_handle;

extern volatile bool carriageHigh; 
extern volatile bool carriageLow;
extern volatile bool clawFullyExtended;
extern volatile bool clawFullyRetracted;

extern double petX;
extern double petY;
extern double petW;
extern double petH;

extern bool closeEnough;
extern bool clawCentered;
extern bool anglePastThreshold;

extern volatile int speed;    // average speed
extern int petsPickedUp;
extern bool rotationTested; // for testing
#include <Arduino.h>
#include "driver/ledc.h"
#include "driver/adc.h"
#include <ESP32Servo.h>

// global variables and task handles

TaskHandle_t drive_handle = NULL;
TaskHandle_t grab_handle = NULL;
TaskHandle_t reverse_handle = NULL;
TaskHandle_t raise_basket_handle = NULL;
TaskHandle_t home_handle = NULL;
TaskHandle_t full_turn_handle = NULL;
TaskHandle_t detect_handle = NULL;

#define leftPwmChannel 0
#define rightPwmChannel 1
// THIS PIN WILL NEED TO BE CHANGED EVENTUALLY SO THAT IT LINES UP WITH SCHEMATIC
#define pwmOut1 13 //outputs the pwm channel according to ledcAttachPin
#define dirOut1 26
#define pwmOut2 4
#define dirOut2 4
#define irSensorLeft 32
#define irSensorRight 33
#define thresholdL 1800
#define thresholdR 1800
#define maxSpeed 4000 //set a max pwm output
#define minSpeed 0 //set a min pwm output
#define speed 1600 //set an average speed
#define SG90Pin 14
#define DSPin 12
#define MG996RPin 13
// other pins: 27 = p_pot, 14 = d_pot

int distance = 0; // right = positive
int last_distance = 0;
int kp=0;
int kd=0;
int p=0;
int d=0;
int m=0; // counts cycles since the reading last changed; more cycles = smaller d
int q=0; // equivalent to m_last (last previous value of m)
int ctrl=0;
int dir1=1;
int dir2=1;
bool leftOnTape=1;
bool rightOnTape=1;
int leftVal=0;
int rightVal=0;
int leftSpeed=0;
int rightSpeed=0;
int lastOnTape = 0; // -1: left; 1: right
uint32_t reverseMultiplier = 0.3;

Servo SG90;
uint32_t SG90Pos = 0;

Servo DS;
uint32_t DSPos = 0;

Servo MG996R;
uint32_t MG996RPos = 0;

// put function declarations here:

/**
 * distToTape - calculates the distance to the tape based on the IR sensor readings
 * 
 * @return an integer indicating the robot's distance from the tape, either +/-5, +/-1, or 0 (on the line)
 */
int distToTape() {
    int dist = 0;
    // leftVal=0;
    // rightVal=0;
    // for (int i=0; i<5; i++) {
    //   leftVal+=adc1_get_raw(ADC1_CHANNEL_4);
    //   rightVal+=adc1_get_raw(ADC1_CHANNEL_5);
    //   delayMicroseconds(500);
    // }
    leftOnTape = adc1_get_raw(ADC1_CHANNEL_4) > thresholdL;
    rightOnTape = adc1_get_raw(ADC1_CHANNEL_5) > thresholdR;
    if (leftOnTape == 1 && rightOnTape == 1) {
        dist = 0;
    } else if (leftOnTape == 0 && rightOnTape == 1) {
        // only right sensor is on tape -> robot is to the left
        dist = -1;
        lastOnTape = 1;
    } else if (leftOnTape == 1 && rightOnTape == 0) {
        // only left sensor is on tape -> robot is to the left
        dist = 1;
        lastOnTape = -1;
    } else if (leftOnTape == 0 && rightOnTape == 0 && lastOnTape == 1) {
        // neither on tape but right was last one on tape
        dist = -5;
    } else {
        // neither on tape but left was last one on tape
        dist = 5;
    }
    return dist;
}

void drive(int avgSpeedInput) {
    last_distance=distance;
    distance = distToTape();
    

    
    if(last_distance != distance) {
        q=m;
        m=1;
    }

    adc2_get_raw(ADC2_CHANNEL_7, ADC_WIDTH_10Bit, &kp); //10 bit: cap at 1024
    adc2_get_raw(ADC2_CHANNEL_6, ADC_WIDTH_10Bit, &kd);
    p=kp*distance;
    d=(int) ((float)kd*(float)(distance - last_distance)/(float)(q+m));
    // i+=ki*distance;
    ctrl=(int)(p+d); // should be several hundred
    m++; 
    
    digitalWrite(dirOut1, dir1);
    digitalWrite(dirOut2, dir2);
    leftSpeed = max(avgSpeedInput-ctrl,minSpeed);
    leftSpeed = min(leftSpeed,maxSpeed);
    rightSpeed = max(avgSpeedInput+ctrl,minSpeed);
    rightSpeed = min(rightSpeed,maxSpeed);
    ledcWrite(leftPwmChannel,leftSpeed);
    ledcWrite(rightPwmChannel,rightSpeed);

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

// create tasks here --> main robot functions

void drive_task(void* parameters) {
  //this creates an infinite loop, but it will be interrupted by other actions
  for(;;) {

    drive(speed);

    // the vTaskDelay function takes in ticks as a time measurement, so / portTick_PERIOD_MS converts to ms
    vTaskDelay(4 / portTICK_PERIOD_MS);
  }
}

void grab_task(void* paramters) {
  // grabbing code here, feel free to change parameters
}

void reverse_task(void* parameters) {
  
  // waits for third switch press before intiating reverse + basket raise (first hit at the door, second when going up the ramp, final on the zipline)
  uint32_t pressCount = 0;
  while (pressCount < 3) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    pressCount++;
  }

  for(;;) {
    // reverse code for aligning with zipline
    vTaskSuspend(&drive_handle);
    // reverse direction and drive slowly backwards (depending on reverseMultiplier) for two seconds
    dir1 = 0;
    dir2 = 0;
    digitalWrite(dirOut1, dir1);
    digitalWrite(dirOut2, dir2);
    ledcWrite(leftPwmChannel,maxSpeed* reverseMultiplier);
    ledcWrite(rightPwmChannel,maxSpeed* reverseMultiplier);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    //stop motors for basket raise
    ledcWrite(leftPwmChannel,0);
    ledcWrite(rightPwmChannel,0);

    //signals to start basket raising code
    xTaskNotifyGive(&raise_basket_handle);
  }
}

void raise_basket_task(void* parameters) {
  
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  for(;;) {
  // raise basket by rotating SG90 by 90 degrees slowly (hence the delays)
    while(MG996RPos < 90) {
      MG996R.write(++MG996RPos);
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
  }
}

void home_task(void* parameters) {
  // homing sequence, to be run once at startup and then deleted
}

void full_turn_task(void* parameters) {
  // full 180 turn sequence here, for once 90 second hard limit reached
}

void detect_task(void* parameters) {
  // detection code for determining pet location
}

// add more functions here

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // initialize adc channels for pwm signals to operate drive
  adc1_config_width(ADC_WIDTH_BIT_12);

  adc1_config_channel_atten(ADC1_CHANNEL_4,ADC_ATTEN_DB_12); // ir sensor inputs (pin 32)
  adc1_config_channel_atten(ADC1_CHANNEL_5,ADC_ATTEN_DB_12); //pin 33
  adc2_config_channel_atten(ADC2_CHANNEL_7, ADC_ATTEN_DB_12); // pin 27 = p_pot (to be removed later)
  adc2_config_channel_atten(ADC2_CHANNEL_6, ADC_ATTEN_DB_12); // pin 14 = d_pot

  ledcSetup(leftPwmChannel,250,12); //middle number: duty cycle resolution in hz
  ledcSetup(rightPwmChannel,250,12);
  ledcAttachPin(pwmOut1,leftPwmChannel);
  ledcAttachPin(pwmOut2,rightPwmChannel); //both motors controlled by same pwm channel

  // create tasks associated with functions defined above 
  // priorities are temporary and TBD
  xTaskCreate(
    drive_task, // function to be run
    "Driving", // description of task
    1000, // bytes allocated to this stack
    NULL, // parameters, dependent on function
    1, // priority
    &drive_handle // task handle
  ); 

  xTaskCreate(
    grab_task, // function to be run
    "Grabbing", // description of task
    1000, // bytes allocated to this stack
    NULL, // parameters, dependent on function
    3, // priority
    &grab_handle // task handle
  ); 

  xTaskCreate(
    reverse_task, // function to be run
    "Reversing", // description of task
    1000, // bytes allocated to this stack
    NULL, // parameters, dependent on function
    3, // priority  
    &reverse_handle // task handle
  ); 

  xTaskCreate(
    raise_basket_task, // function to be run
    "Raising Basket", // description of task
    1000, // bytes allocated to this stack
    NULL, // parameters, dependent on function
    1, // priority
    &raise_basket_handle // task handle
  ); 

  xTaskCreate(
    home_task, // function to be run
    "Homing", // description of task
    1000, // bytes allocated to this stack
    NULL, // parameters, dependent on function
    1, // priority
    &home_handle // task handle
  ); 

  xTaskCreate(
    full_turn_task, // function to be run
    "Turning (360)", // description of task
    1000, // bytes allocated to this stack
    NULL, // parameters, dependent on function
    3, // priority
    &detect_handle // task handle
  );

  xTaskCreate(
    detect_task, // function to be run
    "Detecting", // description of task
    1000, // bytes allocated to this stack
    NULL, // parameters, dependent on function
    2, // priority
    &detect_handle // task handle
  ); 
  
  // Servo setups
  SG90.setPeriodHertz(50);
  SG90.attach(SG90Pin,500,2400);

  DS.setPeriodHertz(50);
  DS.attach(DSPin,500,2400);

  MG996R.setPeriodHertz(50);
  MG996R.attach(MG996RPin,500,2400);


}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
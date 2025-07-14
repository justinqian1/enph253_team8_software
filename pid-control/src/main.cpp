// motor PID control

#include <Arduino.h>
#include "driver/ledc.h"
#include "driver/adc.h"

#define leftPwmChannel 0
#define rightPwmChannel 1
#define pwmOut1 13 //outputs the pwm channel according to ledcAttachPin
//#define dirOut1 26
#define pwmOut2 4
//#define dirOut2 4
#define irSensorLeft 32
#define irSensorRight 33
#define thresholdL 1800
#define thresholdR 1800
#define maxSpeed 4000 //set a max pwm output
#define minSpeed 0 //set a min pwm output
#define speed 1600 //set an average speed
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

void setup() {
    Serial.begin(115200);
    adc1_config_width(ADC_WIDTH_BIT_12);

    adc1_config_channel_atten(ADC1_CHANNEL_4,ADC_ATTEN_DB_12); // ir sensor inputs (pin 32)
    adc1_config_channel_atten(ADC1_CHANNEL_5,ADC_ATTEN_DB_12); //pin 33
    adc2_config_channel_atten(ADC2_CHANNEL_7, ADC_ATTEN_DB_12); // pin 27 = p_pot (to be removed later)
    adc2_config_channel_atten(ADC2_CHANNEL_6, ADC_ATTEN_DB_12); // pin 14 = d_pot

    ledcSetup(leftPwmChannel,250,12); //middle number: duty cycle resolution in hz
    ledcSetup(rightPwmChannel,250,12);
    ledcAttachPin(pwmOut1,leftPwmChannel);
    ledcAttachPin(pwmOut2,rightPwmChannel); //both motors controlled by same pwm channel
    //pinMode(dirOut1,OUTPUT);
    //pinMode(dirOut2,OUTPUT);
}

/**
 * distToTape - calculate the distance from the tape based on IR sensor readings
 * 
 * @return an integer that indicates distance to the tape, either +/- 5, +/- 1, or 0 (on the line)
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

void loop() {
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
    //i+=ki*distance;
    ctrl=(int)(p+d); // should be several hundred
    m++; 
    
    //digitalWrite(dirOut1, dir1);
    //digitalWrite(dirOut2, dir2);
    leftSpeed = max(speed-ctrl,minSpeed);
    leftSpeed = min(leftSpeed,maxSpeed);
    rightSpeed = max(speed+ctrl,minSpeed);
    rightSpeed = min(rightSpeed,maxSpeed);
    ledcWrite(leftPwmChannel,leftSpeed);
    ledcWrite(rightPwmChannel,rightSpeed);

    leftVal = adc1_get_raw(ADC1_CHANNEL_4);
    rightVal = adc1_get_raw(ADC1_CHANNEL_5);
    
    Serial.print("Left reading:");
    Serial.println(leftVal);
    Serial.print("Right reading:");
    Serial.println(rightVal);
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
    delay(4);
}

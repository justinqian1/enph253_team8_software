#include <Arduino.h>
#include "driver/ledc.h"
#include "driver/adc.h"

// global variables and task handles

TaskHandle_t drive_handle = NULL;
TaskHandle_t grab_handle = NULL;
TaskHandle_t reverse_handle = NULL;
TaskHandle_t raise_basket_handle = NULL;
TaskHandle_t home_handle = NULL;

// put function declarations here:


// create tasks here --> main robot functions

void drive(void* parameters) {
  //this creates an infinite loop, but it will be interrupted by other actions
  for(;;) {
    // driving code here, mostly PID control
  }
}

void grab(void* paramters) {
  // grabbing code here, feel free to change parameters
}

void reverse(void* parameters) {
  // reverse code for aligning with zipline
}

void raise_basket(void* parameters) {
  // raising basket code here
}

void home(void* parameters) {
  // homing sequence, to be run once at startup and then deleted
}

// add more functions here

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // create tasks associated with functions defined above 
  xTaskCreate(
    drive, // function to be run
    "Driving", // description of task
    1000, // bytes allocated to this stack
    NULL, // parameters, dependent on function
    1, // priority
    &drive_handle // task handle
  ); 

  xTaskCreate(
    grab, // function to be run
    "Grabbing", // description of task
    1000, // bytes allocated to this stack
    NULL, // parameters, dependent on function
    1, // priority
    &grab_handle // task handle
  ); 

  xTaskCreate(
    reverse, // function to be run
    "Reversing", // description of task
    1000, // bytes allocated to this stack
    NULL, // parameters, dependent on function
    1, // priority
    &reverse_handle // task handle
  ); 

  xTaskCreate(
    raise_basket, // function to be run
    "Raising Basket", // description of task
    1000, // bytes allocated to this stack
    NULL, // parameters, dependent on function
    1, // priority
    &raise_basket_handle // task handle
  ); 

  xTaskCreate(
    home, // function to be run
    "Homing", // description of task
    1000, // bytes allocated to this stack
    NULL, // parameters, dependent on function
    1, // priority
    &home_handle // task handle
  ); 
  
  
  
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
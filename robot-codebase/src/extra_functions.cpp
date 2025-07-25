// #include <hardware/Motor.h>
// #include <constants.h>

// /**
//  * driveReverse - drives the robot in reverse in a straight line without PID control
//  *
//  * @avgSpeedInput the speed at which to drive backwards
//  */
// void driveReverse(const int avgSpeedInput)
// {
//     driveMotor(leftPwmChannel, dirOut1, avgSpeedInput, 0);
//     driveMotor(rightPwmChannel, dirOut2, avgSpeedInput, 0);
// }

// /**
//  * stopDrive - stops both motors entirely
//  *              no guarantees about stopping motion of robot entirely (backlash, momentum, etc...)
//  */
// void stopDrive()
// {
//     ledcWrite(leftPwmChannel, 0);
//     ledcWrite(rightPwmChannel, 0);
// }

// /**
//  * Drives a specific motor and a given speed in a given direction, as opposed to drive which drives both wheel motors
//  * @param motorPWM the PWM channel for the motor
//  * @param directionPin the direction pin for the motor
//  * @param direction the direction to drive, with 1 being right and 0 being left
//  * @param speed the speed to drive the motor, in range 0 - 4095
//  */
// void driveMotor(const int motorPWM, const int directionPin, const int speed, const int direction)
// {
//     digitalWrite(directionPin, direction);
//     ledcWrite(motorPWM, speed);
// }

// /**
//  * Stops a specific motor
//  * @param motorPWM the PWM channel for the motor
//  */
// void stopMotor(const int motorPWM)
// {
//     ledcWrite(motorPWM, 0);
// }

// /**
//  * Stops all motors (generally for testing purposes)
//  */
// void stopAllMotors()
// {
//     ledcWrite(leftPwmChannel, 0);
//     ledcWrite(rightPwmChannel, 0);
//     ledcWrite(clawExtPWMChannel, 0);
//     ledcWrite(carriageHeightPWMChannel, 0);
// }

// // This is an ISR implementation of the button press interrupt for the reversing and basket raising mechanism
// // it sends a notification to the reverse_task to commence (after three presses)
// void IRAM_ATTR basketSwitchPressedISR()
// {
//     BaseType_t hpw = pdFALSE;
//     vTaskNotifyGiveFromISR(reverse_handle, &hpw);
//     portYIELD_FROM_ISR(&hpw);
// }

// // for communication with Pi, I was thinking the detect and grab tasks would bounce between each other
// /**
//  * this task operates the grabbing system of the robot
//  * @param parameters no parameters for this task
//  */
// void grab_task(void *parameters)
// {

//     ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

//     // grabbing code here, feel free to change parameters
// }

// /**
//  * this task operates the reverse driving of the robot, limited to straight-line motion. This task also covers alignment
//  * with the zipline by reversing and then sending a signal to the raise_basket task.
//  * @param parameters no parameters for this task
//  */
// void reverse_task(void *parameters)
// {

//     // waits for third switch press before intiating reverse + basket raise (first hit at the door, second when going up the ramp, final on the zipline)
//     uint32_t pressCount = 0;
//     while (pressCount < 3)
//     {
//         ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//         pressCount++;
//     }

//     for (;;)
//     {
//         // reverse code for aligning with zipline
//         vTaskSuspend(&drive_handle);
//         // reverse direction and drive slowly backwards (depending on reverseMultiplier) for two seconds
//         driveReverse(speed * reverseMultiplier);
//         vTaskDelay(2000 / portTICK_PERIOD_MS);
//         // stop motors for basket raise
//         stopDrive();

//         // signals to start basket raising code
//         xTaskNotifyGive(&raise_basket_handle);
//     }
// }

// /**
//  * this task operates the raising of the basket for the zipline. It suspends the drive task temporarily while raising
//  * the basket.
//  * @param parameters no parameters for this task
//  */
// void raise_basket_task(void *parameters)
// {

//     ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//     stopDrive();
//     // raise basket by rotating SG90 by 90 degrees slowly (hence the delays)
//     // there used to be a for(;;) loop here but I didn't really see why there was one so I removed it.
//     while (DSPos < 90)
//     {
//         MG996R.write(++DSPos);
//         vTaskDelay(10 / portTICK_PERIOD_MS);
//     }
// }

// /**
//  * this task operates a full 180 degree turn of the robot, ideally without losing the line. It is a safeguard for the
//  * case in which the 90-second limit is reached. It suspends all other tasks during its operation, and only resumes
//  * the drive task when completed.
//  * @param parameters no parameters for this task
//  */
// void full_turn_task(void *parameters)
// {
//     // full 180 turn sequence here, for once 90 second hard limit reached
//     ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

//     // abort all other actions
//     vTaskSuspendAll();

//     // hard code in 90-degree turn

//     // drive back to the start  (hopefully 180 turn lines up with tape)
//     vTaskResume(&drive_handle);
// }

// void test_drive(void *parameters) {
//     DriveMotors* robot = static_cast<DriveMotors*>(parameters);
//     for (;;) {
//         Serial.println("before switch");
//         robot->driveLeftMotor(speed,HIGH);
//         Serial.println(leftMotor->currentDirection);
//         vTaskDelay(1000);
//         Serial.println("after switch");
//         robot->driveRightMotor(speed,LOW);
//         vTaskDelay((1000/pwmFreq) / portTICK_PERIOD_MS);
//     }
// }

// //setup
// if (run) {
//         // initialize UART connection to the Pi
//         Serial2Pi.begin(115200, SERIAL_8N1, RXPin, TXPin);
//         Serial2Pi.write("Hello from the ESP32!");

//         // initialize basic pin connections

//         // needs to be pull up for encoder to function properly (I think <-- TO BE TESTED)
//         pinMode(rotaryEncoderPinA, INPUT_PULLUP);
//         pinMode(rotaryEncoderPinB, INPUT_PULLUP);

//         // initialize adc channels for pwm signals to operate drive
//         adc1_config_width(ADC_WIDTH_BIT_12);

//         adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_12); // ir sensor inputs (pin 32)
//         adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_12); // pin 33
//         adc2_config_channel_atten(ADC2_CHANNEL_7, ADC_ATTEN_DB_12); // pin 27 = p_pot (to be removed later)
//         adc2_config_channel_atten(ADC2_CHANNEL_6, ADC_ATTEN_DB_12); // pin 14 = d_pot

//         ledcSetup(leftPwmChannel, 250, 12); // middle number: duty cycle resolution in hz
//         ledcSetup(rightPwmChannel, 250, 12);
//         ledcAttachPin(pwmOut1, leftPwmChannel);
//         ledcAttachPin(pwmOut2, rightPwmChannel); // both motors controlled by same pwm channel

//         ledcSetup(carriageHeightPWMChannel, 250, 12);
//         ledcAttachPin(carriageMotorPWM, carriageHeightPWMChannel);

//         ledcSetup(clawExtPWMChannel, 250, 12);
//         ledcAttachPin(clawExtMotorPWM, clawExtPWMChannel);

//         // attach pins for ISRs

//         pinMode(startSwitch, INPUT_PULLUP);
//         attachInterrupt(digitalPinToInterrupt(startSwitch), startButtonPressedISR, RISING);
//         pinMode(basketSwitch, INPUT_PULLUP);
//         attachInterrupt(digitalPinToInterrupt(basketSwitch), basketSwitchPressedISR, RISING);

//         // starts the PCNT setup code
//         PCNTSetup();
//         // create tasks associated with functions defined above
//         // priorities are temporary and TBD
//         xTaskCreate(
//             drive_task,   // function to be run
//             "Driving",    // description of task
//             1000,         // bytes allocated to this ib_deps = madhephaestus/ESP32Servo@^3.0.8stack
//             NULL,         // parameters, dependent on function
//             1,            // priority
//             &drive_handle // task handle
//         );

//         xTaskCreate(
//             grab_task,   // function to be run
//             "Grabbing",  // description of task
//             1000,        // bytes allocated to this stack
//             NULL,        // parameters, dependent on function
//             2,           // priority
//             &grab_handle // task handle
//         );

//         xTaskCreate(
//             reverse_task,   // function to be run
//             "Reversing",    // description of task
//             1000,           // bytes allocated to this stack
//             NULL,           // parameters, dependent on function
//             3,              // priority
//             &reverse_handle // task handle
//         );

//         xTaskCreate(
//             raise_basket_task,   // function to be run
//             "Raising Basket",    // description of task
//             1000,                // bytes allocated to this stack
//             NULL,                // parameters, dependent on function
//             3,                   // priority
//             &raise_basket_handle // task handle
//         );

//         // high priority task since it occurs on startup
//         xTaskCreate(
//             home_task,   // function to be run
//             "Homing",    // description of task
//             1000,        // bytes allocated to this stack
//             NULL,        // parameters, dependent on function
//             5,           // priority
//             &home_handle // task handle
//         );

//         // high priority task since it overrides all other functions once 90 seconds are triggered
//         xTaskCreate(
//             full_turn_task,  // function to be run
//             "Turning (360)", // description of task
//             1000,            // bytes allocated to this stack
//             NULL,            // parameters, dependent on function
//             5,               // priority
//             &detect_handle   // task handle
//         );

//         xTaskCreate(
//             detect_task,   // function to be run
//             "Detecting",   // description of task
//             1000,          // bytes allocated to this stack
//             NULL,          // parameters, dependent on function
//             2,             // priority
//             &detect_handle // task handle
//         );

//         xTaskCreate(
//             idle_task,   // function to be run
//             "Idling",    // description of task
//             1000,        // bytes allocated to this stack
//             NULL,        // parameters, dependent on function
//             5,           // priority
//             &idle_handle // task handle
//         );

//         // Servo setups
//         SG90.setPeriodHertz(50);
//         SG90.attach(SG90Pin, 500, 2400);

//         DS.setPeriodHertz(50);
//         DS.attach(DSPin, 500, 2400);
//     }
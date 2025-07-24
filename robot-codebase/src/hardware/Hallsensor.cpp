#include <Arduino.h>
#include "Hallsensor.h"
#include "constants.h"  /* For voltage ref and threshold*/

/* serial.println(analogread(halleffect))
*/


HallSensor::HallSensor(int pin) : hallPin(pin) {
    pinMode(pin, INPUT);
    analogReadResolution(12); // Set ADC resolution to 12 bits
}

/* Read voltage from the analog pin (scaled from raw ADC)*/
double HallSensor::readVoltage() {
    int sensorValue = adc1_get_raw(ADC1_CHANNEL_0); // pin 36
    double voltage = sensorValue * (hallVoltageRef / 4095.0); /* Convert to voltage*/
    return voltage;
}

/* Return true if voltage indicates magnetic field presence*/
bool HallSensor::magnetDetected(double voltage) {
    return voltage < magnetThresholdVoltage;
}
/*
/* Read voltage, check for magnet, and format a log message*/
/*
String HallSensor::senseAndLog() {
    float hallVoltage = readVoltage();
    bool magnet_detected = magnetDetected(hallVoltage);
    String logMessage = "[Hall] Voltage: " + String(hallVoltage, 2) +
                        " V, Magnet: " + (magnet_detected ? "Yes" : "No");
    return logMessage;
}*/


#include <Arduino.h>
#include "Hallsensor.h"
#include "constants.h"  /* For voltage ref and threshold*/

/* serial.println(analogread(halleffect))
*/


HallSensor::HallSensor(int pin) : hallPin(pin) {
    pinMode(pin, INPUT);               // Use the instance variable
    analogReadResolution(12);
}

double HallSensor::readVoltage() {
    int sensorValue = analogRead(hallPin); // Read from the correct pin
    float voltage = sensorValue * (HALL_VOLTAGE_REF / 4095.0);
    return voltage;
}


/* Return true if voltage indicates magnetic field presence*/
bool HallSensor::magnetDetected(double voltage) {
    return (voltage < MAGNET_THRESHOLD_VOLTAGE) ;
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


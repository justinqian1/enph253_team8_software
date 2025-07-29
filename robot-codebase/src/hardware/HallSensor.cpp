#include <Arduino.h>
#include "HallSensor.h"
#include "constants.h"  /* For voltage ref and threshold*/

/* serial.println(analogread(halleffect))
*/


HallSensor::HallSensor(int pin) : hallPin(pin) {
      pinMode(HALL_SENSOR_PIN, INPUT);
  analogReadResolution(12); // Set ADC resolution to 12 bits
}

/* Read voltage from the analog pin (scaled from raw ADC)*/
float HallSensor::readVoltage() {
    int sensorValue = analogRead(HALL_SENSOR_PIN); /* Read raw analog value*/
    float voltage = sensorValue * (HALL_VOLTAGE_REF / 4095.0); /* Convert to voltage*/
    return voltage;
}

/* Return true if voltage indicates magnetic field presence*/
bool HallSensor::magnetDetected(float voltage) {
      if (voltage < MAGNET_THRESHOLD_VOLTAGE) { 
    return true; 
  } 
  return false; 
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


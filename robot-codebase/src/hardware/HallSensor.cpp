#include <Arduino.h>
#include "HallSensor.h"
#include "constants.h"  /* For voltage ref and threshold*/

/* serial.println(analogread(halleffect))
*/


HallSensor::HallSensor(adc1_channel_t hallChannel) : _adcChannel(hallChannel) {
    adc1_config_width(ADC_WIDTH_12Bit); // 0–4095 resolution
    adc1_config_channel_atten(_adcChannel, ADC_ATTEN_DB_12); // GPIO32
}

double HallSensor::readVoltage() {
    int sensorValue = analogRead(_adcChannel); // Read from the correct pin
    float voltage = sensorValue * (HALL_VOLTAGE_REF / 4095.0);
    return voltage;
}


/* Return true if voltage indicates magnetic field presence*/
bool HallSensor::magnetDetected(double voltage) {
    return (voltage < MAGNET_THRESHOLD_VOLTAGE) ;
}

/* Read voltage, check for magnet, and format a log message*/
/*
String HallSensor::senseAndLog() {
    float hallVoltage = readVoltage();
    bool magnet_detected = magnetDetected(hallVoltage);
    String logMessage = "[Hall] Voltage: " + String(hallVoltage, 2) +
                        " V, Magnet: " + (magnet_detected ? "Yes" : "No");
    return logMessage;
}*/


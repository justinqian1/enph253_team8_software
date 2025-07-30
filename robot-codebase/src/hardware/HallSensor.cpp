#include <Arduino.h>
#include "HallSensor.h"
#include "constants.h"  /* For voltage ref and threshold*/

HallSensor::HallSensor(adc1_channel_t adcCh) : _adcChannel(adcCh) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(adcCh, ADC_ATTEN_DB_12);
}

/* Read voltage from the analog pin (scaled from raw ADC)*/
double HallSensor::readVoltage() {
    int sensorValue = adc1_get_raw(this->_adcChannel); // pin 36
    double voltage = sensorValue * (hallVoltageRef / 4095.0); /* Convert to voltage*/
    return voltage;
}

/* Return true if voltage indicates magnetic field presence*/
bool HallSensor::magnetDetected(double voltage) {
    return voltage < magnetThresholdVoltage;
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


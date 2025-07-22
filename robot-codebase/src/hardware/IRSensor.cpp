//
// Created by bram on 21/07/25.
//

#include "IRSensor.h"

bool IRSensor::_adcInitialized = false;

IRSensor::IRSensor(adc1_channel_t channel) : _adcChannel(channel) {
    if (!_adcInitialized) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        _adcInitialized = true;
    }
    adc1_config_channel_atten(channel, ADC_ATTEN_DB_12);
}

int IRSensor::read() {
    return adc1_get_raw(_adcChannel);
}

int IRSensor::readThreshold(int threshold) {
    return (adc1_get_raw(_adcChannel) > threshold);
}


adc1_channel_t IRSensor::getChannel() {
    return _adcChannel;
}




//
// Created by bram on 21/07/25.
//

#ifndef IRSENSOR_H
#define IRSENSOR_H

#include <Arduino.h>
#include <driver/adc.h>

class IRSensor {
public:
    /**
     * Creates an IR Sensor object associated with a specific ADC channel, and configures the ADC if not already configured
     * @param channel the adc channel to read from
     */
    IRSensor(adc1_channel_t channel);

    /**
     * reads the reflectance value
     * @return the reflectance value from 0-4095
     */
    int read();

    /**
     * returns 1 if the reading if above the threshold value, and 0 otherwise
     * @param threshold the threshold reading value from 0 to 4095
     * @return
     */
    int readThreshold(int threshold);

    /**
     * returns the ADC channel associated with this IR sensor
     * @return the ADC channel associated with this IR sensor
     */
    adc1_channel_t getChannel();



private:
    int _IRPin = -1;
    adc1_channel_t _adcChannel;
    static bool _adcInitialized;
};

#endif //IRSENSOR_H

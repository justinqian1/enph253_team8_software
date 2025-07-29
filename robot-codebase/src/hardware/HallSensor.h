
#ifndef HALLSENSOR_H
#define HALLSENSOR_H
#include "constants.h"
#include <driver/adc.h>

class HallSensor {
    public:
/* Set pin mode and ADC resolution */
HallSensor(int pin);
// HallSensor(adc_channel_t adcCh);

/* Read voltage from the analog pin (scaled from raw ADC) */
double readVoltage();

/* Return true if voltage is below threshold */
bool magnetDetected(double voltage);

/* Format sensor voltage and detection into a readable message */
//String HallSensor::senseAndLog();

    private:
    int hallPin;
    adc1_channel_t _adcChannel;
};
#endif //HALLSENSOR_H


#ifndef HALLSENSOR_H
#define HALLSENSOR_H
#include "constants.h"
class HallSensor {
    public:
/* Set pin mode and ADC resolution */
HallSensor(int pin);

/* Read voltage from the analog pin (scaled from raw ADC) */
float readVoltage();

/* Return true if voltage is below threshold */
bool magnetDetected(float voltage);

/* Format sensor voltage and detection into a readable message */
//String HallSensor::senseAndLog();

    private:
    int hallPin;
};
#endif HALLSENSOR_H

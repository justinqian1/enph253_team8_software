#include <Arduino.h>
#include <HardwareSerial.h>

HardwareSerial MySerial(1);
bool petDetected=0;
float pet_x=0;
float pet_y=0;
float pet_w=0;
float pet_h=0;

void setup() {
    Serial.begin(115200);
    MySerial.begin(115200,SERIAL_8N1,9,10);
}

void loop() {
    if (MySerial.available()) {
        String line = MySerial.readStringUntil('\n');
        if (line.length()==1) {
            petDetected=0;
            Serial.printf("no pets\n");
        } else {
            petDetected=1;
            sscanf(line.c_str(), "%f,%f,%f,%f", &pet_x, &pet_y, &pet_w, &pet_h);
            Serial.printf("x=%.2f y=%.2f w=%.2f h=%.2f\n", pet_x, pet_y, pet_w, pet_h);
        }
    }
}

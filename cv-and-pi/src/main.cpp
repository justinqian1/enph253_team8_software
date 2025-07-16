#include <Arduino.h>
#include <HardwareSerial.h>

HardwareSerial MySerial(1);
bool inMessage=0;
int petsLeft=0;

void setup() {
    Serial.begin(115200);
    MySerial.begin(115200,SERIAL_8N1,9,10);
}

void loop() {
    if (MySerial.available()) {
        String line = MySerial.readStringUntil('\n');
        if (line=="START") {
            inMessage=1;
            petsLeft=0;
            Serial.printf("new image\n");
        } else if (line=="END") {
            inMessage=0;
        } else if(inMessage && petsLeft==0) {
            petsLeft=line.toInt();
        } else if(inMessage && petsLeft > 0) {
            float x, y, w, h;
            sscanf(line.c_str(), "%f,%f,%f,%f", &x, &y, &w, &h);
            Serial.printf("BBox: x=%.2f y=%.2f w=%.2f h=%.2f\n", x, y, w, h);
            petsLeft--;
        }
    }
}

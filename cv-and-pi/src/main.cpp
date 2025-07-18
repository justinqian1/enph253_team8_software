#include <Arduino.h>
#include <HardwareSerial.h>
#include <ESP32Servo.h>

HardwareSerial MySerial(0);
bool petDetected=0;
float petX=0;
float petY=0;
float petW=0;
float petH=0;
constexpr int MG996RPin = 20; //pin #

constexpr int img_size=320;
constexpr float horizontal_fov=62.2;
constexpr int angleForward=90;
constexpr int angleThreshold=75;
volatile int speed=1600;
int i = 0;

Servo MG996R;
uint32_t MG996Pos = 90;

void rotateTurret(int angle) //NEEDS UPDATING
{
    int newAngle=MG996R.read() + angle;
    newAngle=min(newAngle,180);
    newAngle=max(newAngle,0);
    MG996R.write(newAngle);
}

float angleToCenter(float pet_x_coord) {
    return (pet_x_coord-(float)img_size/2)*horizontal_fov;
}

void pickUpPet() {
    sleep(1);
}

void setup() {
    MySerial.begin(115200,SERIAL_8N1,1,3);
    MG996R.setPeriodHertz(50);
    MG996R.attach(MG996RPin,500,2400);
    MySerial.println("Serial starting");
}

void loop() {
    if (MySerial.available()) {
        String line = MySerial.readStringUntil('\n');
        if (line.length()==1) {
            // no pets; continue as normal
            petDetected=0;
            //Serial.printf("no pets\n");
        } else {
            // pet in visual range
            float rotate_const=0.5;
            petDetected=1;
            sscanf(line.c_str(), "%f,%f,%f,%f", &petX, &petY, &petW, &petH);

            //rotate turret
            if (abs(petX-img_size/2) > 20) { //tolerance of 20; center of pet in pixels 140-180 is good enough
                rotateTurret((int)(angleToCenter(petX)*rotate_const));
            }

            // slow down robot/initiate pick up sequence
            float petArea = petW*petH;
            if (petArea > 1500) { //within ~2ft of pet
                int currentAngle = MG996R.read();
                if (currentAngle < angleForward - angleThreshold || currentAngle > angleForward + angleThreshold) {
                    // arm is at nearly 90 degree angle -> initiate pick up
                    //stopMotors();
                    //vTaskSuspendAll();
                    pickUpPet();      
                    //xTaskResumeAll();                  
                } else {
                    // not close enough - update speed
                    int tempSpeedCeiling = (int)((float)5000*exp(-0.0008*petArea)); // arbitrary function for now, decreases speed as pet draws closer
                    int currentSpeed = speed;
                    tempSpeedCeiling=max(tempSpeedCeiling,100); // make sure it's positive
                    speed=min(currentSpeed,tempSpeedCeiling);
                }
            }
            //Serial.printf("x=%.2f y=%.2f w=%.2f h=%.2f\n", pet_x, pet_y, pet_w, pet_h);
        }
    }
}


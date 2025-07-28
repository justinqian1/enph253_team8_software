#include <Arduino.h>
#include <HardwareSerial.h>
#include <ESP32Servo.h>
#include <hardware/CustomServo.h>

HardwareSerial MySerial(0);
double petX=0;
double petY=0;
double petW=0;
double petH=0;

//booleans for pick up
bool closeEnough = false;
bool clawCentered = false;
bool anglePastThreshold = false;

//thresholds for pick up
constexpr int angleThreshold=75;
constexpr int clawCenterThreshold=20; //px
constexpr double areaThresholdForPickup=5000.0;

constexpr int imgSize=320;
constexpr double horizontal_fov=62.2;
constexpr int angleForward=90;
volatile int speed=1600;
constexpr int pwmChannel=0;
constexpr int MG996RPin = 13; //pin #
constexpr int servoFreq = 50;
constexpr int minDuty = 500;
constexpr int maxDuty = 2500;

//debug rotation
bool rotationTested=false;

CustomServo MG996R(MG996RPin,pwmChannel,angleForward,servoFreq,minDuty,maxDuty, 1.316);
uint32_t MG996Pos = 55;

void resetVars() {
    MG996R.rotateTo(120);
    closeEnough = false;
    clawCentered = false;
    anglePastThreshold = false;
    speed=1900;
}

int servoPosToAngle(int pos) {
    return (int)(round(((double)pos-15.0)*9.0/4.0));
}

int angleToServoPos (int angle) {
    return (int)(round(4.0/9.0*(double)angle)+15);
}

void rotateTurret(int angle)
{
    int newAngle=servoPosToAngle(MG996Pos) + angle;
    newAngle=constrain(newAngle,0,180);
    MG996Pos=angleToServoPos(newAngle);
}

double angleToCenter(double pet_x_coord) {
    double result=(pet_x_coord-(double)imgSize/2)/(double)imgSize*horizontal_fov;
    MySerial.printf("Turret off by: %.2lf\n",result);
    return result;
}

void pickUpPet() {
    MySerial.print("Pickup sequence initiated\n");
    delay(1000);
    resetVars();
}

void testRotation() {
    //MG996R.rotateTo(120);
    
    MG996R.rotateTo(120);
    MySerial.println("position set to 120");
    delay(1500);

    MG996R.rotateBy(90);
    MySerial.println("position should be 210");
    delay(1500);

    MG996R.rotateBy(-90);
    MySerial.println("position should be 120");
    delay(1500);

    MG996R.rotateBy(-90);
    MySerial.println("position should be 30");
    delay(1500);

    MG996R.rotateBy(90);
    MySerial.println("position should be 120");
    MySerial.println("test ended");
        
   /*
    bool increasing=true;
    MG996Pos = 80;
    int count = 0;
    while (1) {
        if (MG996Pos>=140) {
            increasing=false;
        } else if (MG996Pos<=30) {
            increasing=true;
        }

        if (increasing) {
            MG996Pos++;
        } else {
            MG996Pos--;
        }

        if (MG996Pos==55) {
            count++;
        }

        ledcWrite(pwmChannel,MG996Pos);
        Serial.println(MG996Pos);
        //MySerial.println(MG996Pos);
        delay(60);
    }
        */
}

void setup() {
    MySerial.begin(115200,SERIAL_8N1,3,1);
    MySerial.println("Serial starting");
    //MG996R.setPeriodHertz(50);
    //MG996R.attach(MG996RPin,500,2500);
    //ledcSetup(pwmChannel, 50, 10); 
    //ledcAttachPin(MG996RPin,pwmChannel);
    //Serial.begin(115200);
    //Serial.println("serial starting");
}

void loop() {
    
    if (!rotationTested) {
       testRotation();
       rotationTested=true;
    }
    //delay(1000);
    //testRotation();
    
    //ledcWrite(pwmChannel,MG996Pos);
    if (MySerial.available()) {
        String line = MySerial.readStringUntil('\n');
        if (line=="[SYSTEM MESSAGE] RESET") {
            resetVars();
            //rotationTested=false;
            MySerial.printf("System message 'RESET' received\n");
        } else if (line.length()>1) {
            // pet in visual range AND large enough (done on pi)
            sscanf(line.c_str(), "%lf,%lf,%lf,%lf", &petX, &petY, &petW, &petH);
            MySerial.printf("ESP received: x=%.2lf y=%.2lf w=%.2lf h=%.2lf\n", petX, petY, petW, petH);

            // get pet area and print current angle
            double petArea = petW*petH;
            int currentAngle=MG996R.getPosition();
            MySerial.printf("servo angle: %d\n",currentAngle);

            //check if pet big enough for pickup
            closeEnough = petArea > areaThresholdForPickup;           

            // check if angle is correct (off forward direction by at least 75 deg)
            anglePastThreshold = (currentAngle < angleForward - angleThreshold ||
                                  currentAngle > angleForward + angleThreshold);

            //rotate turret
            double rotate_const=1.0;
            if (abs((int)petX-imgSize/2) > clawCenterThreshold) {
                MG996R.rotateBy((int)(angleToCenter(petX)*rotate_const));
                //rotateTurret((int)(angleToCenter(petX)*rotate_const));
                clawCentered=false;
            } else {
                clawCentered = true;
            }

            // check if ready for pickup
            MySerial.printf("Claw centered: %d\n",clawCentered);
            MySerial.printf("Pet close enough: %d\n",closeEnough);
            MySerial.printf("Angle past threshold: %d\n",anglePastThreshold);

            if (clawCentered && closeEnough && anglePastThreshold) {
                // arm is at nearly 90 degree angle -> initiate pick up
                //stopMotors();
                //vTaskSuspendAll();
                pickUpPet();      
                //xTaskResumeAll();   
            } else {
                // not close enough - update speed
                int tempSpeedCeiling = (int)(-0.25*petArea+2300.0); // arbitrary function for now, decreases speed as pet draws closer
                int currentSpeed = speed;
                tempSpeedCeiling=max(tempSpeedCeiling,1000); // make sure speed is positive
                speed=min(currentSpeed,tempSpeedCeiling);
                MySerial.printf("robot speed: %d\n",speed);
            }
        }
    }
    delay(10);
}
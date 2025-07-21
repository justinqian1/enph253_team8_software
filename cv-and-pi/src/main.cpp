#include <Arduino.h>
#include <HardwareSerial.h>
#include <ESP32Servo.h>

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
constexpr double areaThresholdForPickup=4000.0;


constexpr int img_size=320;
constexpr double horizontal_fov=62.2;
constexpr int angleForward=90;
volatile int speed=1600;
constexpr int pwmChannel=0;
constexpr int MG996RPin = 21; //pin #


Servo MG996R;
uint32_t MG996Pos = 55;

int bound(int var, int minVal, int maxVal) {
    if (minVal > maxVal) {
        throw std::invalid_argument("minVal cannot be greater than maxVal");
    }
    int result = min(var, maxVal);
    return max(result, minVal);
}

double bound(double var, double minVal, double maxVal) {
    if (minVal > maxVal) {
        throw std::invalid_argument("minVal cannot be greater than maxVal");
    }
    double result = min(var, maxVal);
    return max(result, minVal);
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
    newAngle=bound(newAngle,0,180);
    MG996Pos=angleToServoPos(newAngle);
}

double angleToCenter(double pet_x_coord) {
    double result=(pet_x_coord-(double)img_size/2)/(double)img_size*horizontal_fov;
    MySerial.printf("Turret rotating, input: %.2lf, off by: %.2lf\n",pet_x_coord,result);
    return result;
}

void pickUpPet() {
    MySerial.print("Pickup sequence initiated\n");
    sleep(1);
}

bool increasing=true;

void testRotation() {
    if (MG996Pos>=95) {
        increasing=false;
    } else if (MG996Pos<=15) {
        increasing=true;
    }
    if (increasing) {
        MG996Pos++;
    } else {
        MG996Pos--;
    }
    ledcWrite(pwmChannel,MG996Pos);
    MySerial.println(MG996Pos);
    delay(50);
}

void setup() {
    MySerial.begin(115200,SERIAL_8N1,3,1);
    MG996R.setPeriodHertz(50);
    MG996R.attach(MG996RPin,500,2500);
    ledcSetup(pwmChannel, 50, 10); 
    ledcAttachPin(MG996RPin,pwmChannel);
    MySerial.println("Serial starting");
}
void loop() {
    //delay(1000);
    //MG996R.write(55);
    //MySerial.print("still running\n");
    //testRotation();
    ledcWrite(pwmChannel,MG996Pos);
    if (MySerial.available()) {
        String line = MySerial.readStringUntil('\n');
        if (line.length()>1) {
            // pet in visual range AND large enough (done on pi)
            sscanf(line.c_str(), "%lf,%lf,%lf,%lf", &petX, &petY, &petW, &petH);
            MySerial.printf("ESP received: x=%.2lf y=%.2lf w=%.2lf h=%.2lf\n", petX, petY, petW, petH);

            // slow down robot/initiate pick up sequence
            double petArea = petW*petH;
            int currentAngle = servoPosToAngle(MG996Pos);
            MySerial.printf("servo angle: %d\n",currentAngle);

            //rotate turret
            double rotate_const=0.7;
            if (abs((int)petX-img_size/2) > clawCenterThreshold) {
                rotateTurret((int)(angleToCenter(petX)*rotate_const));
                MySerial.printf("new servo angle once rotated: %d\n",servoPosToAngle(MG996Pos));
                clawCentered=false;
            } else {
                clawCentered = true;
            }

            //check if pet big enough for pickup
            closeEnough = petArea > areaThresholdForPickup;           

            // check if angle is correct (off forward direction by at least 75 deg)
            anglePastThreshold = (currentAngle < angleForward - angleThreshold ||
                                  currentAngle > angleForward + angleThreshold);

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
                int tempSpeedCeiling = (int)(5000.0*exp(-0.0008*petArea)); // arbitrary function for now, decreases speed as pet draws closer
                int currentSpeed = speed;
                tempSpeedCeiling=max(tempSpeedCeiling,100); // make sure speed is positive
                speed=min(currentSpeed,tempSpeedCeiling);
                MySerial.printf("robot speed: %d",speed);
            }
        }
    }
    delay(10);
}
#include "functions.h"
#include "constants.h"
#include "globals.h"

void resetVars() {
    MG996R->rotateTo(carriageForwardPos);
    closeEnough = false;
    clawCentered = false;
    anglePastThreshold = false;
    speed=defaultSpeed;
}

/**
 * calculates angle to center of pet. Note that the input image is flipped vertically.
 * @param pet_x_coord center of pet's x coordinate
 * @return angle between -31 (pet on very left of frame) to +31 (pet on very right of frame)
 */
double angleToCenter(double petX) {
    double result= (petX-(double)imgSize/2)/(double)imgSize*horizontal_fov;
    Serial2Pi.printf("Turret off by: %.2lf\n",result);
    return result;
}

void setupLimitSwitches() {
    pinMode(carriageLOW, INPUT_PULLUP);
    pinMode(carriageHIGH, INPUT_PULLUP);
    pinMode(clawExtendedSwitch, INPUT_PULLUP);
    pinMode(clawRetractedSwitch, INPUT_PULLUP);
}

/** 
 * changes carriage height
 * @param up, true if moving up and false if moving down
 */
void moveCarriage(bool up) {
    carriageMotor->driveMotor(carriageSpeed,up);
    Serial2Pi.println(up ? "Moving carriage upwards" : "Moving carriage downwards");
    uint32_t switchToPoll;
    up ? switchToPoll = CARRIAGE_HIGH_SWITCH : switchToPoll = CARRIAGE_LOW_SWITCH;
    pollSwitch(switchToPoll);
}

void extendClaw (bool outwards) {
    uint32_t switchToPoll;
    if (outwards) {
        Serial2Pi.println("Claw extending");
        clawFullyRetracted = false;
        clawExtMotor->driveForward(clawExtSpeed);
        switchToPoll = CLAW_EXT_SWITCH;
    } else {
        Serial2Pi.println("Claw retracting");
        clawFullyExtended = false;
        clawExtMotor->driveReverse(clawExtSpeed);
        switchToPoll = CLAW_RET_SWITCH;
    }
    pollSwitch(switchToPoll);
}

void closeClaw(bool close) {
    if (close) {
        SG90->rotateTo(clawClosedPos);
        Serial2Pi.println("claw closing");
    } else {
        SG90->rotateTo(clawOpenPos);
        Serial2Pi.println("claw opening");
    }
}

/**
 * picks up pet
 */
void pickUpPet() {
    vTaskSuspend(detect_handle);
    bool targetHeight = heightsForPickup[petsPickedUp];
    if (targetHeight && !carriageHigh) {
        moveCarriage(UP);
        //xTaskNotify(raise_carriage_handle, targetHeight, eSetValueWithOverwrite); // doesn't work rn since we need it to return
    } else if (!targetHeight && !carriageLow) {
        moveCarriage(DOWN);
    }
    extendClaw(true);
    // receive input from hall effect
    closeClaw(true);
    vTaskDelay(1000);
    petsPickedUp++;
    Serial2Pi.printf("Pet picked up!\n");
    //dropPetInBasket();
    return; // START DROP SEQUENCE
}

void dropPetInBasket() {
    if (!carriageHigh) {
        moveCarriage(UP);
        //xTaskNotify(raise_carriage_handle, true, eSetValueWithOverwrite); // moves carriage up if it's low; DOESN'T WORK RN
    }
    
    extendClaw(false); // retract claw
    MG996R->rotateTo(carriageMaxRightPos); //rotate to max angle
    closeClaw(false); // open claw
    vTaskDelay(1500); // give time to drop pet
    extendClaw(true); // re extend claw
    prepareForNextPickup();
}

void prepareForNextPickup() {
    Serial2Pi.println("Preparing for next pickup");
    moveCarriage(UP); // move carriage up
    pickupSide[petsPickedUp] ? MG996R->rotateTo(carriageForwardPos-45) : MG996R->rotateTo(carriageForwardPos+45);

    extendClaw(false);
    vTaskResume(detect_handle);
    //vTaskResume(drive_handle);
}

void testRotation() {    
    MG996R->rotateTo(90);
    Serial2Pi.println("position set to 90");
    delay(1500);

    MG996R->rotateBy(90);
    Serial2Pi.println("position should be 180");
    delay(1500);

    MG996R->rotateBy(-90);
    Serial2Pi.println("position should be 90");
    delay(1500);

    MG996R->rotateBy(-90);
    Serial2Pi.println("position should be 0");
    delay(1500);

    MG996R->rotateBy(90);
    Serial2Pi.println("position should be 90");
    Serial2Pi.println("test ended");
}

/**
 * checks for a specific limit switch being hit
 * @param switch_id the switch id to check
 *                  MUST be between 1-4
 */
bool checkSwitchHit(uint32_t switch_id) {
    bool result;
    switch(switch_id) {
        case CARRIAGE_HIGH_SWITCH:
            result = analogRead(carriageHIGH) > limitSwitchActiveThreshold;
            if (result) {
                carriageHigh=true;
                carriageLow=false;
                carriageMotor->stopMotor();
                //xTaskNotifyGive(raise_carriage_handle);
                Serial.println("Carriage high switch hit");
            }
            break;
        case CARRIAGE_LOW_SWITCH:
            result = analogRead(carriageLOW) > limitSwitchActiveThreshold;
            if (result) {
                carriageHigh=false;
                carriageLow=true;
                carriageMotor->stopMotor();
                //xTaskNotifyGive(raise_carriage_handle);
                Serial.println("Carriage low switch hit");
            }
            break;
        case CLAW_EXT_SWITCH:
            result = analogRead(clawExtendedSwitch) > limitSwitchActiveThreshold;
            if (result) {
                clawFullyExtended=true;
                clawFullyRetracted=false;
                clawExtMotor->stopMotor();
                Serial.println("Claw full extension switch hit");
            }
            break;
        case CLAW_RET_SWITCH:
            result = analogRead(clawRetractedSwitch) > limitSwitchActiveThreshold;
            if (result) {
                clawFullyExtended=false;
                clawFullyRetracted=true;
                clawExtMotor->stopMotor();
                Serial.println("Claw full retraction switch hit");
            }
            break;
        default:
            Serial.println("Error: unknown switch ID");
            return true;
    }
    return result;
}

bool pollSwitch(uint32_t switch_id) {
    int count=0;
    
    if (switch_id < minSwitchID || switch_id > maxSwitchID) {
        // switchToPoll value invalid
        Serial.print("Error: cannot poll switch ");
        Serial.println(switch_id);
        return false;
    }
    //poll switch
    Serial.print("Polling switch ");
    Serial.println(switch_id);
    while(!checkSwitchHit(switch_id)) {
        if (count % (1000/switchPollFrequency) == 0) {
            Serial.println("Still waiting for switch to hit...");
        }
        count++;
        delay(switchPollFrequency);
    }
    return true; // when switch hits
}

/**
 * Runs the homing sequence for the robot
 * NEEDS FULL REWRITE - COMMENTED OUT FOR NOW
 */
void home()
{
    /**
     * Code for homing sequence to run on startup, including:
     * Homing DC motors using limit switches (2 motors)
     * Setting all servo motor positions to 0
     */

    // uint32_t switchHit;
    /*
    SG90Pos = 0;
    DSPos = 0;
    MG996RPos = 0;

    SG90.write(SG90Pos);
    DS.write(DSPos);
    MG996R.write(MG996RPos);
    */

    // find limits of the claw
    // driveAndreMotor(clawExtPwmChannelExt, clawExtPwmChannelRet, homeSpeed, 0);
    // xTaskNotifyWait(0, 0xFFFFFFFF, &switchHit, portMAX_DELAY);
    // if (switchHit == 3)
    // {
    //     pcnt_get_counter_value(PCNT_UNIT, &rotaryMin);
    // }
    // else if (switchHit == 4)
    // {
    //     pcnt_get_counter_value(PCNT_UNIT, &rotaryMax);
    // }

    // driveAndreMotor(clawExtPwmChannelExt, clawExtPwmChannelRet, homeSpeed, 1);
    // xTaskNotifyWait(0, 0xFFFFFFFF, &switchHit, portMAX_DELAY);
    // if (switchHit == 3)
    // {
    //     pcnt_get_counter_value(PCNT_UNIT, &rotaryMin);
    // }
    // else if (switchHit == 4)
    // {
    //     pcnt_get_counter_value(PCNT_UNIT, &rotaryMax);
    // }
    // stopMotor(clawExtPwmChannelExt,clawExtPwmChannelRet);

    // driveAndreMotor(carriageHeightPwmChannelUp, carriageHeightPwmChannelDown, homeSpeed, 0);
    // xTaskNotifyWait(0, 0xFFFFFFFF, &switchHit, portMAX_DELAY);
    // if (switchHit == 1)
    // {
    //     stopMotor(carriageHeightPwmChannelUp,carriageHeightPwmChannelDown);
    // }
    // else if (switchHit == 2)
    // {
    //     driveAndreMotor(carriageHeightPwmChannelUp, carriageHeightPwmChannelDown, homeSpeed, 1);
    //     xTaskNotifyWait(0, 0xFFFFFFFF, &switchHit, portMAX_DELAY);
    //     stopMotor(carriageHeightPwmChannelUp, carriageHeightPwmChannelDown);
    // }
}
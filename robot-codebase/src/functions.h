#include "Arduino.h"

#pragma once

void resetVars();
int distToTape();
double angleToCenter(double petX);
void drive(int avgSpeedInput);
void configIRSensors();
void moveCarriage(bool up);
void extendClaw (bool outwards);
void closeClaw(bool close);
void pickUpPet();
void dropPetInBasket();
void prepareForNextPickup();
bool checkSwitchHit(uint32_t switch_id);
bool pollSwitch(uint32_t switch_id);
void home();
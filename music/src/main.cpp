#include <Arduino.h>

#include "driver/ledc.h"

#define INTERVAL 300
#define DELAY 30
// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  ledcSetup(LEDC_CHANNEL_1, 50, 16);
  ledcAttachPin(13,LEDC_CHANNEL_1);
}

void loop() {
 ledcWriteNote(LEDC_CHANNEL_1, NOTE_D, 6);
 delay(INTERVAL);
 ledcWriteNote(LEDC_CHANNEL_1, NOTE_Bb, 6);
 delay(INTERVAL);
 ledcWriteNote(LEDC_CHANNEL_1,NOTE_G, 5);
 delay(INTERVAL);
 ledcWrite(1,0);
 delay(DELAY);
 ledcWriteNote(1, NOTE_G,5);
 delay(INTERVAL/2);
 ledcWriteNote(1, NOTE_D, 6);
 delay(INTERVAL);
 ledcWrite(1,0);
 delay(DELAY);
 ledcWriteNote(1, NOTE_D,6);
 delay(INTERVAL/2);
 ledcWriteNote(1, NOTE_A, 6);
 delay(INTERVAL);
 ledcWriteNote(1, NOTE_F, 5);
 delay(INTERVAL);
 ledcWrite(1,0);
 delay(DELAY);
 ledcWriteNote(1, NOTE_F, 5);
 delay(INTERVAL/2);
 ledcWriteNote(1,NOTE_C, 5);
 delay(INTERVAL);
 ledcWrite(1,0);
 delay(DELAY);
 ledcWriteNote(1,NOTE_C, 5);
 delay(INTERVAL);
 ledcWriteNote(1, NOTE_E, 5);
 delay(INTERVAL);
 ledcWrite(1,0);
 delay(DELAY);
 ledcWriteNote(1, NOTE_E, 5);
 delay(INTERVAL/2);
 ledcWriteNote(1, NOTE_F, 5);
 delay(INTERVAL/2);

}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}
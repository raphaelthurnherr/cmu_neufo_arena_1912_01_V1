#include <Arduino.h>
#include "cmu_ws_1912_01_V1_board.h"

//#define vMax 3.31
//#define motorStepMax 512

board_1912_01_V01 motor_1912_board;

char button;
unsigned int mVa;
unsigned int mVb;
unsigned int mVc;
unsigned char motorPosPercent = 50;
int channelApercent;

void setup() {
  Serial.begin(9600); // open the serial port at 9600 bps:
  motor_1912_board.begin();
}

void loop() {
 
  motor_1912_board.stepperRotation(0,50,64);
  delay(3000);
  }
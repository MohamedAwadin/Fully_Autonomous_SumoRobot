/********************************************************************************************************************/
/********************************************************************************************************************/
/*******************  Author         : Mohamed Awadin                                           *********************/
/*******************  FILE NAME      : InterruptHandling.cpp                                    *********************/
/*******************  Layer          : Private                                                  *********************/
/*******************  Project        : Fully_Autonomous_SumoRobot                               *********************/
/*******************  Microcontroller: Arduino Uno (or compatible)                              *********************/
/*******************  Version        : 1.00                                                     *********************/
/********************************************************************************************************************/
/********************************************************************************************************************/
#include "InterruptHandling.h"
#include "MotorControl.h"
#include <Arduino.h>

#define FRONT_L_IR_SENSOR 2
#define FRONT_R_IR_SENSOR 21
#define BACK_L_IR_SENSOR 19
#define BACK_R_IR_SENSOR 20

extern volatile boolean detected, lastRight, lastLeft, lastBack, lastForward;

void handleInterrupt() {
  delayMicroseconds(500);
  detected = true;

  if (digitalRead(FRONT_L_IR_SENSOR) == LOW && digitalRead(FRONT_R_IR_SENSOR) == LOW) {
    while (digitalRead(FRONT_L_IR_SENSOR) == LOW && digitalRead(FRONT_R_IR_SENSOR) == LOW) {
      moveBackward(250, 250);
    }
    lastBack = true;
    lastForward = false;
  } else if (digitalRead(BACK_L_IR_SENSOR) == LOW && digitalRead(BACK_R_IR_SENSOR) == LOW) {
    while (digitalRead(BACK_L_IR_SENSOR) == LOW && digitalRead(BACK_R_IR_SENSOR) == LOW) {
      moveForward(250, 250);
    }
    lastBack = false;
    lastForward = true;
  } else if (digitalRead(BACK_L_IR_SENSOR) == LOW && digitalRead(FRONT_L_IR_SENSOR) == LOW) {
    while (digitalRead(BACK_L_IR_SENSOR) == LOW && digitalRead(FRONT_L_IR_SENSOR) == LOW) {
      moveForward(200, 250);
    }
    lastLeft = true;
    lastRight = false;
  } else if (digitalRead(BACK_R_IR_SENSOR) == LOW && digitalRead(FRONT_R_IR_SENSOR) == LOW) {
    while (digitalRead(BACK_R_IR_SENSOR) == LOW && digitalRead(FRONT_R_IR_SENSOR) == LOW) {
      moveForward(250, 200);
    }
    lastLeft = false;
    lastRight = true;
  } else {
    detected = false;
  }
}

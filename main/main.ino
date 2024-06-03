/********************************************************************************************************************/
/********************************************************************************************************************/
/*******************  Author         : Mohamed Awadin                                           *********************/
/*******************  FILE NAME      : main.ino                                                 *********************/
/*******************  Layer          : Application (RUNNABLE&SETUP)                             *********************/
/*******************  Project        : Fully_Autonomous_SumoRobot                               *********************/
/*******************  Microcontroller: Arduino Uno (or compatible)                              *********************/
/*******************  Version        : 1.00                                                     *********************/
/********************************************************************************************************************/
/********************************************************************************************************************/

#include <NewPing.h>
#include <IRremote.h>
#include "MotorControl.h"
#include "SensorHandling.h"
#include "DirectionHandling.h"
#include "InterruptHandling.h"

/********************************/
// Pin Definitions
#define LED1 10
#define IR_RECEIVER_PIN 13

#define FRONT_L_IR_SENSOR 2
#define FRONT_R_IR_SENSOR 21
#define BACK_L_IR_SENSOR 19
#define BACK_R_IR_SENSOR 20

/********************************/
// Global Variables
IRrecv irReceiver(IR_RECEIVER_PIN);
decode_results results;

volatile int distF = 0, distB = 0, distR = 0, distL = 0;
volatile boolean detected = false;
volatile boolean lastRight = false, lastLeft = false, lastBack = false, lastForward = false;
volatile boolean angleRight = false, angleLeft = false;
int startFlag = 0;

/********************************/
// Setup Function
void setup() {
  Serial.begin(9600);
  irReceiver.enableIRIn();
  
  // Initialize motor control pins
  initMotorControl();
  
  // Initialize sensor handling pins
  initSensorHandling();
  
  // Initialize interrupt handling
  attachInterrupt(digitalPinToInterrupt(FRONT_L_IR_SENSOR), handleInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FRONT_R_IR_SENSOR), handleInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BACK_L_IR_SENSOR), handleInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BACK_R_IR_SENSOR), handleInterrupt, CHANGE);
}

/********************************/
// Main Loop
void loop() {
  if (irReceiver.decode(&results)) {
    if (results.value == 0x10EFD827) { // replace with actual start command value
      startFlag = 1;
    }
    irReceiver.resume();
  }

  if (startFlag) {
    updateDistances();
    if (!detected) {
      goToDirection(FRONT);
    }
  }
}

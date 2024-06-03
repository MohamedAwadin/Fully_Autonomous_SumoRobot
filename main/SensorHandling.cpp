/********************************************************************************************************************/
/********************************************************************************************************************/
/*******************  Author         : Mohamed Awadin                                           *********************/
/*******************  FILE NAME      : SensorHandling.cpp                                       *********************/
/*******************  Layer          : HAL                                                      *********************/
/*******************  Project        : Fully_Autonomous_SumoRobot                               *********************/
/*******************  Microcontroller: Arduino Uno (or compatible)                              *********************/
/*******************  Version        : 1.00                                                     *********************/
/********************************************************************************************************************/
/********************************************************************************************************************/


/**
 * @brief 
 * Description: These files are responsible for managing the ultrasonic sensors used for distance measurement. 
 * "SensorHandling.h" declares the functions for initializing the sensors and reading their values, 
 * while "SensorHandling.cpp" provides the implementation for these functions, including the logic to read distances from the ultrasonic sensors.
 * 
 */

#include "SensorHandling.h"
#include <Arduino.h>

extern volatile int distF, distB, distR, distL;
extern volatile boolean detected;

void initSensorHandling() {
  pinMode(SONIC_F_TRIG, OUTPUT);
  pinMode(SONIC_F_ECHO, INPUT);
  pinMode(SONIC_B_TRIG, OUTPUT);
  pinMode(SONIC_B_ECHO, INPUT);
  pinMode(SONIC_R_TRIG, OUTPUT);
  pinMode(SONIC_R_ECHO, INPUT);
  pinMode(SONIC_L_TRIG, OUTPUT);
  pinMode(SONIC_L_ECHO, INPUT);
}

int readUltrasonic(int trigger, int echo) {
  digitalWrite(trigger, LOW);
  delayMicroseconds(5);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  int duration = pulseIn(echo, HIGH);
  return duration / 57;
}

void updateDistances() {
  distF = readUltrasonic(SONIC_F_TRIG, SONIC_F_ECHO);
  if (detected) return;
  distB = readUltrasonic(SONIC_B_TRIG, SONIC_B_ECHO);
  if (detected) return;
  distR = readUltrasonic(SONIC_R_TRIG, SONIC_R_ECHO);
  if (detected) return;
  distL = readUltrasonic(SONIC_L_TRIG, SONIC_L_ECHO);
}


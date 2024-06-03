/********************************************************************************************************************/
/********************************************************************************************************************/
/*******************  Author         : Mohamed Awadin                                           *********************/
/*******************  FILE NAME      : DirectionHandling.cpp                                    *********************/
/*******************  Layer          : Private                                                  *********************/
/*******************  Project        : Fully_Autonomous_SumoRobot                               *********************/
/*******************  Microcontroller: Arduino Uno (or compatible)                              *********************/
/*******************  Version        : 1.00                                                     *********************/
/********************************************************************************************************************/
/********************************************************************************************************************/

/**
 * @brief 
 * Description: These files manage the logic for the robot's movement directions based on sensor inputs.
 * "DirectionHandling.h" declares the functions for determining and setting the robot's direction, 
 * and "DirectionHandling.cpp" implements these functions, defining how the robot should move in response to different direction commands.
 * 
 */
#include "DirectionHandling.h"
#include "MotorControl.h"
#include "SensorHandling.h"
#include <Arduino.h>

extern volatile int distF, distB, distR, distL;
extern volatile boolean detected, lastRight, lastLeft, lastBack, lastForward;

void goToDirection(int dir) {
  switch (dir) {
    case FRONT:
      while ((!detected) && (distF <= MAX_READ) && (distF > 0)) {
        moveForward(255, 255);
        distF = readUltrasonic(SONIC_F_TRIG, SONIC_F_ECHO);
      }
      break;
    case LEFT:
      while ((!detected) && (distF > MAX_READ)) {
        rotateLeft(SEARCH_SPEED);
        distF = readUltrasonic(SONIC_F_TRIG, SONIC_F_ECHO);
      }
      break;
    case RIGHT:
      while ((!detected) && (distF > MAX_READ)) {
        rotateRight(SEARCH_SPEED);
        distF = readUltrasonic(SONIC_F_TRIG, SONIC_F_ECHO);
      }
      break;
    case BACK:
      if (distB < 5) {
        while (distB < 6) {
          distB = readUltrasonic(SONIC_B_TRIG, SONIC_B_ECHO);
          moveForward(255, 150);
        }
      }
      while ((!detected) && (distF > MAX_READ)) {
        rotateLeft(SEARCH_SPEED);
        distF = readUltrasonic(SONIC_F_TRIG, SONIC_F_ECHO);
      }
      break;
    case LAST:
      if (lastBack) {
        moveBackward(200, 200);
        delay(500);
        lastBack = false;
        detected = false;
      } else if (lastForward) {
        moveForward(200, 200);
        delay(500);
        lastForward = false;
        detected = false;
      } else if (lastRight) {
        moveForward(200, 200);
        delay(100);
        lastRight = false;
        detected = false;
      } else if (lastLeft) {
        moveForward(200, 200);
        delay(100);
        lastLeft = false;
        detected = false;
      } else {
        rotateRight(150);
        detected = false;
      }
      break;
  }
}


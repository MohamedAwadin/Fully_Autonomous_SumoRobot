/********************************************************************************************************************/
/********************************************************************************************************************/
/*******************  Author         : Mohamed Awadin                                           *********************/
/*******************  FILE NAME      : MotorControl.cpp                                         *********************/
/*******************  Layer          : HAL                                                      *********************/
/*******************  Project        : Fully_Autonomous_SumoRobot                               *********************/
/*******************  Microcontroller: Arduino Uno (or compatible)                              *********************/
/*******************  Version        : 1.00                                                     *********************/
/********************************************************************************************************************/
/********************************************************************************************************************/

/**
 * @brief 
 * Description: These files handle all motor-related operations. 
 * "MotorControl.h" declares the functions and constants for controlling the motors, 
 * while "MotorControl.cpp" defines the implementation of these functions, such as moving the robot forward, 
 * backward, and rotating left or right.
 * 
 */

#include "MotorControl.h"
#include <Arduino.h>

void initMotorControl() {
  pinMode(ENABLE_RIGHT, OUTPUT);
  pinMode(ENABLE_LEFT, OUTPUT);
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
}

void moveForward(int leftSpeed, int rightSpeed) {
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  analogWrite(ENABLE_LEFT, leftSpeed);

  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  analogWrite(ENABLE_RIGHT, rightSpeed);
}

void moveBackward(int leftSpeed, int rightSpeed) {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
  analogWrite(ENABLE_LEFT, leftSpeed);

  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
  analogWrite(ENABLE_RIGHT, rightSpeed);
}

void rotateRight(int speed) {
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  analogWrite(ENABLE_LEFT, speed);

  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
  analogWrite(ENABLE_RIGHT, speed);
}

void rotateLeft(int speed) {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
  analogWrite(ENABLE_LEFT, speed);

  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  analogWrite(ENABLE_RIGHT, speed);
}

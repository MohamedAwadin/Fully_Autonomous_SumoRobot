/********************************************************************************************************************/
/********************************************************************************************************************/
/*******************  Author         : Mohamed Awadin                                           *********************/
/*******************  FILE NAME      : MotorControl.h                                           *********************/
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

#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#define ENABLE_RIGHT 5
#define ENABLE_LEFT 8
#define LEFT_MOTOR_FORWARD 6
#define LEFT_MOTOR_BACKWARD 7
#define RIGHT_MOTOR_FORWARD 4
#define RIGHT_MOTOR_BACKWARD 3

void initMotorControl();
void moveForward(int leftSpeed, int rightSpeed);
void moveBackward(int leftSpeed, int rightSpeed);
void rotateRight(int speed);
void rotateLeft(int speed);

#endif


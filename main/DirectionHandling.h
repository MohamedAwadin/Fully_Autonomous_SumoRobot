/********************************************************************************************************************/
/********************************************************************************************************************/
/*******************  Author         : Mohamed Awadin                                           *********************/
/*******************  FILE NAME      : DirectionHandling.h                                      *********************/
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
#ifndef DIRECTIONHANDLING_H
#define DIRECTIONHANDLING_H

#define MAX_READ 40 // cm
#define FRONT 0
#define LEFT 1
#define RIGHT 2
#define BACK 3
#define LAST 4
#define SEARCH_SPEED 200

void goToDirection(int dir);

#endif


/********************************************************************************************************************/
/********************************************************************************************************************/
/*******************  Author         : Mohamed Awadin                                           *********************/
/*******************  FILE NAME      : SensorHandling.h                                         *********************/
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


#ifndef SENSORHANDLING_H
#define SENSORHANDLING_H

#define SONIC_F_TRIG 23
#define SONIC_F_ECHO 25
#define SONIC_B_TRIG 27
#define SONIC_B_ECHO 29
#define SONIC_R_TRIG 31
#define SONIC_R_ECHO 33
#define SONIC_L_TRIG 35
#define SONIC_L_ECHO 37

void initSensorHandling();
int readUltrasonic(int trigger, int echo);
void updateDistances();

#endif

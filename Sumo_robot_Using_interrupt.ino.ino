
#include <NewPing.h>
#include <IRremote.h>
/********************************/
#define  LED1           10


/********************************/
int IrReceiverPin = 13;                 // Turn the variable "IrReceiverPin" to pin 22
IRrecv IrReceive(IrReceiverPin);           // create a new instance of "irrecv" and save this instance in variabele "IRrecv"
decode_results results;                 // define the variable "results" to store the received button code

/********************************/
/********MOTORS**********/
#define EnableRight  5             //EN1 > PWM 490hz
#define EnableLeft   8            // EN2 > PWM 980hz
#define LeftMotorForward   6    //  IN3
#define LeftMotorBackward  7   //   IN4
#define RightMotorForward  4  //    IN1
#define RightMotorBackward 3 ///////IN2
//int RIGHT_DC=255;
#define SEARCH_SPEED  200
/******************************/
/*******Ultrasonic**********/
#define sonicF_ECHO    25
#define sonicB_ECHO    29
#define sonicR_ECHO    33
#define sonicL_ECHO    37

#define sonicF_Trig    23
#define sonicB_Trig    27
#define sonicR_Trig    31
#define sonicL_Trig    35

//#define MAX_DISTANCE   150    // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
/*
  NewPing sonicF(sonicF_Trig, sonicF_ECHO, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
  NewPing sonicB(sonicB_Trig, sonicB_ECHO, MAX_DISTANCE);
  NewPing sonicR(sonicR_Trig, sonicR_ECHO, MAX_DISTANCE);
  NewPing sonicL(sonicL_Trig, sonicL_ECHO, MAX_DISTANCE);
*/

volatile int distF = 0, distB = 0, distR = 0, distL = 0;
volatile int MAX_READ = 40; //cm
/******************************/
//connect 4x IR_SENSOR interrupt pins
#define Front_L_irSensor 2
#define Front_R_irSensor 21

#define Back_L_irSensor  19
#define Back_R_irSensor  20

/******************************/
// Direction.
#define FRONT   0
#define LEFT    1
#define RIGHT   2
#define BACK    3
#define LAST    4
/*******************************************/
/*********GLOBAL VAR****************/
volatile boolean Detected = false ;
volatile boolean  Last_Right = false, Last_Left = false , Last_Back = false, Last_Forward = false; //Variable to detect the last move of the motors.
volatile boolean  Angle_Right = false, Angle_Left = false ; //Variable to detect the last  of the motors.
volatile boolean  ROT_Right = false;
int start_flag =0 ;
/*************************************************
 * ***********************************************/
void Interrrupt_FUNC();
//void Interrupt_Behavior();
void Go_TO(int dir) ;
void stopMoving();
void moveForward(int right_dc, int left_dc);
void moveBackward(int right_dc, int left_dc);
void RotatLeft(int speed_dc);
void RotatRight(int speed_dc);
int Sonic(int triger , int echoo);
/*******************************************/



void Interrrupt_FUNC()
{
  delayMicroseconds(500);
  Detected = true;
  //  Serial.println("IR FIRED");
  //    Serial.print("Front_Left:");
  //    Serial.println(digitalRead(Front_L_irSensor));
  //    Serial.print("Front_Right:");
  //    Serial.println(digitalRead(Front_R_irSensor));
  //    Serial.print("Back_Left:");
  //    Serial.println(digitalRead(Back_L_irSensor));
  //    Serial.print("Back_Right:");
  //    Serial.println(digitalRead(Back_R_irSensor));
  if ((digitalRead(Front_L_irSensor) == LOW) && (digitalRead(Front_R_irSensor) == LOW))
  {
    while ((digitalRead(Front_L_irSensor) == LOW) && (digitalRead(Front_R_irSensor) == LOW))
    {
      moveBackward(250, 250); //(RIGHT-LEFT)
    }
    Last_Back = true;
    Last_Forward = false;
  }
  else if ((digitalRead(Back_L_irSensor) == LOW) && (digitalRead(Back_R_irSensor) == LOW))
  {
    while ((digitalRead(Back_L_irSensor) == LOW) && (digitalRead(Back_R_irSensor) == LOW))
    {
      moveForward(250, 250); //(RIGHT-LEFT)
    }
    Last_Back = false;
    Last_Forward = true;
  }
  else if ((digitalRead(Back_L_irSensor) == LOW) && (digitalRead(Front_L_irSensor) == LOW))
  {
    while ((digitalRead(Back_L_irSensor) == LOW) && (digitalRead(Front_L_irSensor) == LOW))
    {
      moveForward(200, 250); //(RIGHT-LEFT)
    }
    Last_Left = true;
    Last_Right = false;
  }
  else if ((digitalRead(Back_R_irSensor) == LOW) && (digitalRead(Front_R_irSensor) == LOW))
  {
    while ((digitalRead(Back_R_irSensor) == LOW) && (digitalRead(Front_R_irSensor) == LOW))
    {
      moveForward(250, 200); //(RIGHT-LEFT)
    }
    Last_Left = false;
    Last_Right = true;
  }

  else if (digitalRead(Front_L_irSensor) == LOW)
  {
    Serial.println("FL");
    while (digitalRead(Front_L_irSensor) == LOW)
    {
      moveBackward(100, 200); //(RIGHT-LEFT)
    }
    Angle_Left = true;
    Angle_Right = false;
  }
  else if (digitalRead(Front_R_irSensor) == LOW)
  {
    Serial.println("FR");
    while (digitalRead(Front_R_irSensor) == LOW)
    {
      moveBackward(200, 100); //(RIGHT-LEFT)
    }
    Angle_Left = true;
    Angle_Right = false;
  }
  else if (digitalRead(Back_L_irSensor) == LOW)
  {
    Serial.println("BL");
    while (digitalRead(Back_L_irSensor) == LOW){
      moveForward(150, 200); //(RIGHT-LEFT)
    }
    Angle_Left = true;
    Angle_Right = false;
  }
  else if (digitalRead(Back_R_irSensor) == LOW)
  {
    Serial.println("BR");
    while (digitalRead(Back_R_irSensor) == LOW)
    {
      moveForward(200, 150); //(RIGHT-LEFT)
    }
    Angle_Left = true;
    Angle_Right = false;
  }
  else {
    Detected = false;
  }
  /*else if (digitalRead(Front_L_irSensor) == LOW)
    {
    while (digitalRead(Front_L_irSensor) == LOW)
    {
      RotatRight(100); //(RIGHT-LEFT)
    }
    Last_Right = true;
    Last_left = false;
    }
    else if (digitalRead(Front_R_irSensor) == LOW)
    {
    while (digitalRead(Front_R_irSensor) == LOW)
    {
      RotatLeft(150); //(RIGHT-LEFT)
    }
    Last_Right = false;
    Last_left = true;
    }
    else if (digitalRead(Back_L_irSensor) == LOW)
    {
    while (digitalRead(Back_L_irSensor) == LOW)
    {
      moveForward(100, 150); //(RIGHT-LEFT)
    }
    Last_Back = false;
    Last_Forward= true;
    }
    else if (digitalRead(Back_R_irSensor) == LOW)
    {
    while (digitalRead(Back_R_irSensor) == LOW)
    {
      moveForward(150, 100); //(RIGHT-LEFT)
    }
     Last_Back = false;
    Last_Forward= true;
    }*/
}

/*******************************************************************************
 *******************************************************************************/
void Go_TO(int dir) {
  int BACK_CONT = 0;
  boolean BACK_OUT = false;
  switch (dir) {
    case FRONT :
      Serial.print("F_FRONT:");
      Serial.println(distF);
      moveForward(255, 255);
      distF = Sonic(sonicF_Trig, sonicF_ECHO);
      while ((!Detected) && (distF <= MAX_READ) && (distF > 0)) {
        Serial.print("Front:");
        Serial.println(distF);

        //delay(50);
        distF = Sonic(sonicF_Trig, sonicF_ECHO);
        distF = Sonic(sonicF_Trig, sonicF_ECHO);
        distF = Sonic(sonicF_Trig, sonicF_ECHO);
        distF = Sonic(sonicF_Trig, sonicF_ECHO);
        //distF = Sonic(sonicF_Trig, sonicF_ECHO);
        //distF = Sonic(sonicF_Trig, sonicF_ECHO);
        moveForward(255, 255);
      }
      //Last_Back = false ;
      break;

    case LEFT :
      Serial.print("LEFT:");
      Serial.println(distL);

      RotatLeft(SEARCH_SPEED);
      distF = Sonic(sonicF_Trig, sonicF_ECHO);
      while ((!Detected) && (distF > MAX_READ) ) {
        Serial.print("F_LEFT:");
        Serial.println(distF);
        RotatLeft(SEARCH_SPEED);
        distF = Sonic(sonicF_Trig, sonicF_ECHO);

        //delay(50);
      }
      //ROT_Right == false;

      break;

    case RIGHT :
      Serial.print("RIGHT:");
      Serial.println(distR);
      RotatRight(SEARCH_SPEED);
      distF = Sonic(sonicF_Trig, sonicF_ECHO);
      while ((!Detected) && (distF > MAX_READ) ) {
        Serial.print("F_RIGHT:");
        Serial.println(distF);
        RotatRight(SEARCH_SPEED);
        distF = Sonic(sonicF_Trig, sonicF_ECHO);

        //delay(50);
      }
      //ROT_Right == true;
      break;

    case BACK :
      Serial.println("BACK:");
      Serial.print(distB);
      //>>>>>>>>>>>>>>>>>>>>>>>>>>>>
      if (distB < 5) {
        BACK_CONT++;
        if (BACK_CONT == 50) {
          while (distB < 6) {
            distB = Sonic(sonicB_Trig, sonicB_ECHO);
            moveForward(255, 150); //(RIGHT,LEFT)
          }
          BACK_CONT = 0;
          BACK_OUT = true;
        }
      }
      //<<<<<<<<<<<<<<<<<<<<<<<<<<<

      RotatLeft(SEARCH_SPEED);
      distF = Sonic(sonicF_Trig, sonicF_ECHO);
      while ((!BACK_OUT)&&(!Detected) && (distF > MAX_READ) ) {
        Serial.println("F_BACK:");
        Serial.print(distF);
        RotatLeft(SEARCH_SPEED);
        distF = Sonic(sonicF_Trig, sonicF_ECHO);

        //delay(50);
      }
      BACK_OUT = false;
      break;
    case LAST :
      if (Last_Back == true) {
        moveBackward(200, 200);
        delay(500);
        Last_Back = false ;
        Detected = false;
      }
      else if (Last_Forward == true) {
        moveForward(200, 200);
        delay(500);
        Last_Forward = false ;
        Detected = false;
      }
      else if (Last_Right == true) {
        moveForward(200, 200);
        delay(100);
        Last_Right = false ;
        Detected = false;
      }
      else if (Last_Left == true) {
        moveForward(200, 200);
        delay(100);
        Last_Left = false ;
        Detected = false;
      }
      else {
        RotatRight(150);
        //stopMoving();
        Detected = false;
      }

      break;
  }
}
/******************************************/
/*******************************************/

void stopMoving()
{
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  analogWrite(EnableRight, 0);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(LeftMotorBackward , LOW);
  analogWrite(EnableLeft, 0);
}
void moveForward(int right_dc, int left_dc)
{
  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(RightMotorBackward, LOW);
  analogWrite(EnableRight, right_dc);
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(LeftMotorBackward , LOW);
  analogWrite(EnableLeft, left_dc);
}

void moveBackward(int right_dc, int left_dc)
{
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(RightMotorBackward, HIGH);
  analogWrite(EnableRight, right_dc);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(LeftMotorBackward , HIGH);
  analogWrite(EnableLeft, left_dc);
}
void RotatLeft(int speed_dc)
{
  int DC_Speed = speed_dc;
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(RightMotorBackward, HIGH);
  analogWrite(EnableRight, DC_Speed);
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(LeftMotorBackward , LOW);
  analogWrite(EnableLeft, DC_Speed);
}

void RotatRight(int speed_dc)
{
  int DC_Speed = speed_dc;
  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(RightMotorBackward, LOW);
  analogWrite(EnableRight, DC_Speed);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(LeftMotorBackward , HIGH);
  analogWrite(EnableLeft, DC_Speed);
}
/***********Ultrasonic Func***************/
int Sonic(int triger , int echoo)
{
  int distance = 0, duration = 0;
  digitalWrite(triger, LOW);
  delayMicroseconds(5);
  digitalWrite(triger, HIGH);
  delayMicroseconds(10);
  digitalWrite(triger, LOW);
  duration = pulseIn(echoo, HIGH);
  distance = duration / 57; //Distance = (Speed of Sound * Time/2) = t/(1/((350*0.0001)/2))
  //distance=duration * 0.0351/2 ;
  return distance;
}
/******************************************************/
void search()
{
  //delay(50);
  /*
    distF = sonicF.ping_cm();
    if (Detected) return;
    distB = sonicB.ping_cm();
    if (Detected) return;
    distR = sonicR.ping_cm();
    if (Detected) return;
    distL = sonicL.ping_cm();
  */
  distF = Sonic(sonicF_Trig, sonicF_ECHO);
  if (Detected) return;
  distB = Sonic(sonicB_Trig, sonicB_ECHO);
  if (Detected) return;
  distR = Sonic(sonicR_Trig, sonicR_ECHO);
  if (Detected) return;
  distL = Sonic(sonicL_Trig, sonicL_ECHO);
}
/******************************************/
/*******************************************/
/*******************************************************************************
   Setup
   This function runs once after reset.
 *******************************************************************************/

void setup() {
  Serial.begin(9600);                 // Initialise the serial monitor

  Serial.println("Starting IR-receiver...");
  IrReceive.enableIRIn();            // start the IR-receiverpinMode(RightMotorForward, OUTPUT);
  Serial.println("IR-receiver active");
  /******************************/
  pinMode(RightMotorForward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
  pinMode(EnableRight, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward , OUTPUT);
  pinMode(EnableLeft, OUTPUT);
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(LeftMotorBackward , LOW);
  /*******************************/
  pinMode(sonicF_Trig, OUTPUT);
  pinMode(sonicF_ECHO, INPUT);
  //%%%%%%%%%%%%%%%%%%%%%%%%
  pinMode(sonicB_Trig, OUTPUT);
  pinMode(sonicB_ECHO, INPUT);
  //%%%%%%%%%%%%%%%%%%%%%%%%
  pinMode(sonicR_Trig, OUTPUT);
  pinMode(sonicR_ECHO, INPUT);
  //%%%%%%%%%%%%%%%%%%%%%%%%
  pinMode(sonicL_Trig, OUTPUT);
  pinMode(sonicL_ECHO, INPUT);
  //%%%%%%%%%%%%%%%%%%%%%%%%
  /******************************/
  pinMode(Front_L_irSensor, INPUT);
  pinMode(Front_R_irSensor, INPUT);
  pinMode(Back_L_irSensor , INPUT);
  pinMode(Back_R_irSensor , INPUT);

  attachInterrupt(digitalPinToInterrupt(Front_L_irSensor), Interrrupt_FUNC, FALLING);
  attachInterrupt(digitalPinToInterrupt(Front_R_irSensor), Interrrupt_FUNC, FALLING);
  attachInterrupt(digitalPinToInterrupt(Back_R_irSensor) , Interrrupt_FUNC, FALLING);
  attachInterrupt(digitalPinToInterrupt(Back_L_irSensor) , Interrrupt_FUNC, FALLING);


  /******************************/
  pinMode(LED1, OUTPUT);


  digitalWrite(LED1 , LOW);

  /******************************/




  // Wait until button is released.

  // Turn on the LEDs.


  //Loop until receive START SIGNAL

  while (start_flag != 1) {
    while (IrReceive.decode(&results) == false);
    // When the IR-receiver receives a signal
    if (results.value == 0xFF42BD) //Button 1
    {
      start_flag = 1;
    }
    IrReceive.resume();
  }
  
  //delay(10000);
  digitalWrite(LED1, HIGH);

  delay(5000);
  
}
/*********************************/
//Main program loop.
void loop() {
  search();

  if (Detected) {
    Serial.println("IR FIRED");
  }

  if ((!Detected) && (distF <= MAX_READ) && (distF > 0)) {
    Go_TO(FRONT);
  }
  else if ((!Detected) && (distL <= MAX_READ) && (distL > 0)) {
    Go_TO(LEFT);
  }
  else if ((!Detected) && (distR <= MAX_READ) && (distR > 0)) {
    Go_TO(RIGHT);
  }
  else if ((!Detected) && (distB <= MAX_READ) && (distB > 0)) {
    Go_TO(BACK);
  }
  else {
    Go_TO(LAST);
  }
}

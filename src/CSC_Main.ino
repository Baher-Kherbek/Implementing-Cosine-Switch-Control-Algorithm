/*
	By Baher Kher Bek
	
	

Calibration Steps:
 * 1- Check Minimum and Maximum RPMs
 * 2- Check Wheels Distance and Wheel's Radius
 * 3- Check Motor's Polarity
 * 
 */

#include <ros.h>
#include <geometry_msgs/Point.h>

// Motor Pins
#define enA 4 // left motor
#define in1 5
#define in2 6
#define enB 9 // right motor
#define in3 7
#define in4 8
#define interruptLeft 2
#define interruptRight 3

// Constants
#define MAXSPEED 120
#define Kp 1
#define Ki 0
#define Kd 0
#define PPR 620


bool prevInterval = true;
int DesiredSpeedLeft, DesiredSpeedRight;

//Interrupt Variables
volatile int CurrentSpeedLeft, CurrentSpeedRight = 0;
volatile float timeStepLeft, timeNowLeft, timePrevLeft = 0;
volatile float timeStepRight, timeNowRight, timePrevRight = 0;

float previousTimeLeft = 0, previousTimeRight = 0, IntegralLeft = 0.0, IntegralRight = 0.0, DerivativeLeft, DerivativeRight;
int PreviousErrorLeft, PreviousErrorRight;
int PWMLeft = 0, PWMRight = 0, errorLeft, errorRight;

//ROS Callback Function
void callback(const geometry_msgs :: Point& msg){

  //Polarity
  if(msg.x > 0 && msg.y > 0) Forward();
  else if (msg.x > 0 && msg.y < 0) CW();
  else if (msg.x < 0 && msg.y > 0) CCW();
  else Backward();

  DesiredSpeedLeft = abs(msg.x);
  DesiredSpeedRight = abs(msg.y);
  
}

//ROS
ros::NodeHandle nh;
ros :: Subscriber <geometry_msgs:: Point> sub("/Control", callback);

void LeftMotorISR(){
  timeNowLeft = micros();
  timeStepLeft = timeNowLeft - timePrevLeft;
  timePrevLeft = timeNowLeft;
  CurrentSpeedLeft = 60 * 1000000 / (PPR * timeStepLeft);
  
}

void RightMotorISR(){
  timeNowRight = micros();
  timeStepRight = timeNowRight - timePrevRight;
  timePrevRight = timeNowRight;
  CurrentSpeedRight = 60 * 1000000 / (PPR * timeStepRight);
  
}

int PIDLeft(int Error){
  int currentTime = millis();
  float elapsedTime = (double)(currentTime - previousTimeLeft);  
                                
  IntegralLeft += Error * elapsedTime;              
  DerivativeLeft = (Error - PreviousErrorLeft) / elapsedTime;  
  int outputSignal = Kp*Error + Ki*IntegralLeft + Kd*DerivativeLeft;                  

  PreviousErrorLeft = Error;                                
  previousTimeLeft = currentTime; 
  
}

int PIDRight(int Error){
  int currentTime = millis();
  float elapsedTime = (double)(currentTime - previousTimeRight);  
                                
  IntegralRight += Error * elapsedTime;              
  DerivativeRight = (Error - PreviousErrorRight) / elapsedTime;  
  int outputSignal = Kp*Error + Ki*IntegralRight + Kd*DerivativeRight;                  

  PreviousErrorRight = Error;                                
  previousTimeRight = currentTime; 
  
}


void setup()
{
  //Pins
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  attachInterrupt(digitalPinToInterrupt(interruptLeft), LeftMotorISR, RISING);
  attachInterrupt(digitalPinToInterrupt(interruptRight), RightMotorISR, RISING);

  //ROS
  nh.initNode(); //Initiate Node
  nh.subscribe(sub); // Subscribe to topic
}

void loop(){
  //Calculate error
  errorLeft = DesiredSpeedLeft - CurrentSpeedLeft;
  errorRight = DesiredSpeedRight - CurrentSpeedRight;
  
  //PID Control on Both Motors
  PWMLeft += PIDLeft(errorLeft);
  PWMRight += PIDRight(errorRight);

  analogWrite(PWMLeft, enB);
  analogWrite(PWMRight, enA);
  
  nh.spinOnce();
  
}



void Forward(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void Backward(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void CCW(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void CW(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

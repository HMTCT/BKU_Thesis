/*
Script to move my tiny 6dof robotic arm
*/
#include <math.h>
#include "kinematics.h"
#include "readSerial.h"
#include "moving.h"

#define PI 3.1415926535897932384626433832795

//driver for the axis 1
#define PUL1_PIN 39
#define DIR1_PIN 37
//driver for the axis 2
#define PUL2_PIN 43
#define DIR2_PIN 41
//driver for the axis 3
#define PUL3_PIN 47
#define DIR3_PIN 45
//driver for the axis 4
#define PUL4_PIN 46
#define DIR4_PIN 48
//driver for the axis 5
#define PUL5_PIN A6
#define DIR5_PIN A7
//driver for the axis 6
#define PUL6_PIN A0
#define DIR6_PIN A1

//enable pin for the axis 3, 2 and 1
#define EN321_PIN 32
#define EN4_PIN A8
#define EN5_PIN A2
#define EN6_PIN 38

const float Xhome[6]= {164.5, 0.0, 241.0, 90.0, 180.0, -90.0}; //{x, y, z, ZYZ Euler angles}
float Xcurr[6] = {164.5, 0.0, 241.0, 90.0, 180.0, -90.0}; //current pos value
float Jhome[6]; //joints value at home
float Jcurr[6]; //current joints value 


void setup()
{
  pinMode(PUL1_PIN, OUTPUT);
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(PUL2_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(PUL3_PIN, OUTPUT);
  pinMode(DIR3_PIN, OUTPUT);
  pinMode(PUL4_PIN, OUTPUT);
  pinMode(DIR4_PIN, OUTPUT);
  pinMode(PUL5_PIN, OUTPUT);
  pinMode(DIR5_PIN, OUTPUT);
  pinMode(PUL6_PIN, OUTPUT);
  pinMode(DIR6_PIN, OUTPUT);

  pinMode(EN321_PIN, OUTPUT);
  pinMode(EN4_PIN, OUTPUT);
  pinMode(EN5_PIN, OUTPUT);
  pinMode(EN6_PIN, OUTPUT);
  
  digitalWrite(PUL1_PIN, LOW); // gear ratio = 96/20 = 4.8
  digitalWrite(DIR1_PIN, LOW); //LOW = negative direction
  
  digitalWrite(PUL2_PIN, LOW); // gear ratio = 4
  digitalWrite(DIR2_PIN, LOW); //LOW = positive direction
  
  digitalWrite(PUL3_PIN, LOW); // gear ratio = 5
  digitalWrite(DIR3_PIN, LOW); //LOW = negative direction
  
  digitalWrite(PUL4_PIN, LOW); // gear ratio = 56/20 = 2.8
  digitalWrite(DIR4_PIN, LOW); //LOW = positive direction
  
  digitalWrite(PUL5_PIN, LOW); // gear ratio = 42/20 = 2.1
  digitalWrite(DIR5_PIN, LOW); //LOW = positive direction
  
  digitalWrite(PUL6_PIN, LOW); // gear ratio = 1
  digitalWrite(DIR6_PIN, LOW); //LOW = positive direction

  // all joints disabled!
  digitalWrite(EN321_PIN, HIGH);
  digitalWrite(EN4_PIN, HIGH);
  digitalWrite(EN5_PIN, HIGH);
  digitalWrite(EN6_PIN, HIGH); 

  //First, go to the home position (all joints equal to 0)
  delay(5000);
  go_home();
  setCurPos(0, 0, 0, 0, 90, 0);

  //Init Jcurr base on Xcurr (Xhome)
  float tmp[6];
  ArrayCopy(Xcurr, tmp);
  InverseK(tmp, Jcurr);
  delay(1000);

  Serial.begin(9600);
}

void loop()
{
  
  //--------------------------------------------------------GoGoGo-------------------
  //testing();
  readSerial();
  excuteCommand();
}

void go_home(){
  // enable all joints
  digitalWrite(EN321_PIN, LOW);
  digitalWrite(EN4_PIN, LOW);
  digitalWrite(EN5_PIN, LOW);
  digitalWrite(EN6_PIN, LOW);
  // joint #2
  singleJointMove(DIR2_PIN, HIGH, PUL2_PIN, 5582);
  // joint #3
  singleJointMove(DIR3_PIN, LOW, PUL3_PIN, 6569);
  // joint #5
  singleJointMove(DIR5_PIN, HIGH, PUL5_PIN, (int)(180/dl5));
  Serial.println("Arm go home");
}

void sleep()
{
  InverseK(Xcurr, Jcurr);
  // go to home position first
  goStrightLine(Jcurr, Jhome, 0.25e-4, 0.75e-10, 0.0, 0.0);
  // come back from home position to fold position
  // joint #5
  singleJointMove(DIR5_PIN, LOW, PUL5_PIN, (int)(90/dl5));
  // joint #3
  singleJointMove(DIR3_PIN, HIGH, PUL3_PIN, 6569);
  // joint #2
  singleJointMove(DIR2_PIN, LOW, PUL2_PIN, 5582);

  // all joints disabled!
  digitalWrite(EN321_PIN, HIGH);
  digitalWrite(EN4_PIN, HIGH);
  digitalWrite(EN5_PIN, HIGH);
  digitalWrite(EN6_PIN, HIGH); 
  // wait 15 minutes
  delay(900000);
}


void testing()
{
  float X1[6]={164.5, 0.0, 141.0, 90.0, 180.0, -90.0};
  float X11[6]={164.5+14.7, 35.4, 141.0, 90.0, 180.0, -90.0};
  float X12[6]={164.5+50.0, 50.0, 141.0, 90.0, 180.0, -90.0};
  float X13[6]={164.5+85.3, 35.4, 141.0, 90.0, 180.0, -90.0};
  float X14[6]={164.5+100.0, 0.0, 141.0, 90.0, 180.0, -90.0};
  float X15[6]={164.5+85.3, -35.4, 141.0, 90.0, 180.0, -90.0};
  float X16[6]={164.5+50.0, -50.0, 141.0, 90.0, 180.0, -90.0};
  float X17[6]={164.5+14.7, -35.4, 141.0, 90.0, 180.0, -90.0};

  float X18[6]={164.5+50.0, 0.0, 141.0, 90.0, 180.0, -90.0};
  
  float X2[6]={264.5, 0.0, 141.0, 0.0, 90.0, 0.0};
  float X3[6]={164.5, 100.0, 141.0, 90.0, 90.0, 0.0};
  float X4[6]={164.5, -100.0, 141.0, 90.0, -90.0, 0.0};
  
  float J1[6], J11[6], J12[6], J13[6], J14[6], J15[6], J16[6], J17[6], J18[6], J2[6], J3[6], J4[6];
  
  //Store pos array
  ArrayCopy(X1, Xcurr);
  
  InverseK(Xhome, Jhome);
  InverseK(X1, J1);
  InverseK(X11, J11);
  InverseK(X12, J12);
  InverseK(X13, J13);
  InverseK(X14, J14);
  InverseK(X15, J15);
  InverseK(X16, J16);
  InverseK(X17, J17);
  InverseK(X18, J18);
  InverseK(X2, J2);
  InverseK(X3, J3);
  InverseK(X4, J4);

  goStrightLine(Jhome, J1, 0.25e-4, 0.75e-10, 0.0, 0.0);

  float velG=0.25e-4;
  goStrightLine(J1, J11, 0.25e-4, 0.75e-10, 0.0, 0.5*velG);
  goStrightLine(J11, J12, 0.25e-4, 0.75e-10, 0.5*velG, 0.5*velG);
  goStrightLine(J12, J13, 0.25e-4, 0.75e-10, 0.5*velG, 0.5*velG);
  goStrightLine(J13, J14, 0.25e-4, 0.75e-10, 0.5*velG, 0.5*velG);
  goStrightLine(J14, J15, 0.25e-4, 0.75e-10, 0.5*velG, 0.5*velG);
  goStrightLine(J15, J16, 0.25e-4, 0.75e-10, 0.5*velG, 0.5*velG);
  goStrightLine(J16, J17, 0.25e-4, 0.75e-10, 0.5*velG, 0.5*velG);
  goStrightLine(J17, J1, 0.25e-4, 0.75e-10, 0.5*velG, 0.0);

  goStrightLine(J1, J18, 0.25e-4, 0.75e-10, 0.0, 0.8*velG);
  goStrightLine(J18, J14, 0.25e-4, 0.75e-10, 0.8*velG, 0.0);
  goStrightLine(J14, J1, 0.25e-4, 0.75e-10, 0.0, 0.0);
  
  goStrightLine(J1, J2, 0.25e-4, 0.75e-10, 0.0, 0.0);
  goStrightLine(J2, J1, 0.25e-4, 0.75e-10, 0.0, 0.0);

  goStrightLine(J1, J3, 0.25e-4, 0.75e-10, 0.0, 0.0);
  goStrightLine(J3, J1, 0.25e-4, 0.75e-10, 0.0, 0.0);

  goStrightLine(J1, J4, 0.25e-4, 0.75e-10, 0.0, 0.0);
  goStrightLine(J4, J1, 0.25e-4, 0.75e-10, 0.0, 0.0);
  //--------------------------------------------------------GoGoGoBack---------------
  // come back from home position to fold position
  sleep();
}

void ArrayCopy(float* src, float* dst){
  for (int i = 0; i < 6; i++){
    dst[i] = src[i];
  }
}

void printArray(float* arr, uint8_t n){
  for (int i = 0; i < n; ++i){
    Serial.println(arr[i]);
  }
}




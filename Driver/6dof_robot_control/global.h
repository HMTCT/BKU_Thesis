#ifndef __GLOBAL__H
#define __GLOBAL__H

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
#define EN4_PIN   A8
#define EN5_PIN   A2
#define EN6_PIN   38

#define PUMP_PIN  63

//{x, y, z, ZYZ Euler angles} for Home position
const float Xhome[6] = { 164.5, 0.0, 241.0, 90.0, 180.0, -90.0 };

//Angle per step move = microstep / gear_ratio
const double dl1 = 360.0/200.0/32.0/4.8;
const double dl2 = 360.0/200.0/32.0/4.0;
const double dl3 = 360.0/200.0/32.0/5.0;
const double dl4 = 360.0/200.0/16.0/2.8;
const double dl5 = 360.0/200.0/16.0/2.1;
const double dl6 = 360.0/200.0/16.0/1.0;

const double DL[6] = {dl1, dl2, dl3, dl4, dl5, dl6};

const float MAX_JOINT_ANGLE[5] = {80, 80, 60, 90, 90};
const float MIN_JOINT_ANGLE[5] = {-80, -80, -80, -90, -90};

//Debug flag
#define DEBUG

/*
250 <= x <= 220
-160 <= y <= 160
49 <= z <= 292
*/

#endif
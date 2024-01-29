#include "readSerial.h"
#include "kinematics.h"
#include "moving.h"

//−−−−−−−−−−VARIABLES USED FOR CONTROLLING ARM−−−−−−−−−−//
const uint8_t PUL_PIN[6] = {PUL1_PIN, PUL2_PIN, PUL3_PIN, PUL4_PIN, PUL5_PIN, PUL6_PIN};
const uint8_t DIR_PIN[6] = {DIR1_PIN, DIR2_PIN, DIR3_PIN, DIR4_PIN, DIR5_PIN, DIR6_PIN};
const double DL[6] = {dl1, dl2, dl3, dl4, dl5, dl6};
float currJoint[6] = {0, 0, 0, 0, 90, 0};

#define ANGLE_PER_COMMAND      2
const char POSITIVE_DIRECTION = '1';
const char NEGATIVE_DIRECTION = '2';

void goHomeFromManual(){
  float tmp[6];
  ArrayCopy(currJoint, tmp);  //Store current joints
  for (int i = 0; i < 6; ++i){
      currJoint[i] = 0;
      //Rotate Joint5 90 degree
      if (i == 4){
        currJoint[i] = 90;
      }
  }

  simultaneousMove(tmp, currJoint, 0.25e-4, 0.75e-10, 0.0, 0.0);
  //Reset Xcurr and Jcurr
  //Init Jcurr base on Xcurr (Xhome)
  ArrayCopy(Xhome, Xcurr);
  
  ArrayCopy(Xcurr, tmp);
  InverseK(tmp, Jcurr);
  setCurPos(0, 0, 0, 0, 90, 0);
}

void goFoldFromManual(){
  goHomeFromManual();
  // come back from home position to fold position
  // joint #5
  singleJointMove(DIR5_PIN, LOW, PUL5_PIN, (int)(180 / dl5));
  // joint #3
  singleJointMove(DIR3_PIN, HIGH, PUL3_PIN, 6569);
  // joint #2
  singleJointMove(DIR2_PIN, LOW, PUL2_PIN, 5582);
}

void singleJointMove(uint8_t DIR_PIN, uint8_t DIR, uint8_t PUL_PIN, int totSteps, int delValue = 4000, int incValue = 7, int accRate = 530)
{
  digitalWrite(DIR_PIN, DIR);
  for (int i = 0; i < totSteps; i++)
  {
   if (totSteps > (2*accRate + 1)){
      if (i < accRate){
        //acceleration
        delValue = delValue - incValue;
      } else if (i > (totSteps - accRate)){
        //decceleration
        delValue = delValue + incValue;
      }
    } else {
      //no space for proper acceleration/decceleration
      if (i < ((totSteps - (totSteps % 2))/2)){
        //acceleration
        delValue = delValue - incValue;
      } else if (i > ((totSteps + (totSteps % 2))/2)){
        //decceleration
        delValue = delValue + incValue;
      }
    }
    digitalWrite(PUL_PIN, HIGH);
    delayMicroseconds(delValue);
    digitalWrite(PUL_PIN, LOW);
    delayMicroseconds(delValue);
  }
}

void simultaneousMove(float* Jcurr, float* Jnext, float vel0, float acc0, float velini, float velfin){
  goStrightLine(Jcurr, Jnext, vel0, acc0, velini, velfin);
  ArrayCopy(Jnext, Jcurr);
}

#include "kinematics.h"
#include "moving.h"
#include "global.h"

//−−−−−−−−−−VARIABLES USED FOR CONTROLLING ARM−−−−−−−−−−//
#define POSITIVE_DIRECTION '1'
#define NEGATIVE_DIRECTION '2'
#define ANGLE_PER_COMMAND   2

const float velG = 0.25e-4;
float start_vel = 1 * velG;
float end_vel = 1 * velG;

#define NUM_BYTES_BUFFER    (6 * sizeof(float))

ArmMoving::ArmMoving(){
  this->cmd = IDLE;
  this->listener = SerialCommunication();
  memset(this->currJoint, 0, NUM_BYTES_BUFFER);
  memset(this->buffer, 0, NUM_BYTES_BUFFER);
}

void ArmMoving::wakeUp(){
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
  singleJointMove(DIR5_PIN, HIGH, PUL5_PIN, (int)(180 / dl5));
  //Serial.println("Arm go home");

  memset(this->currJoint, 0, NUM_BYTES_BUFFER);
  this->currJoint[4] = 90;
  setCurPos(0, 0, 0, 0, 90, 0);
  memcpy(this->buffer, this->currJoint, NUM_BYTES_BUFFER);
}

void ArmMoving::goHomeFromManual(){
  float tmp[6];
  memcpy(tmp, this->currJoint, NUM_BYTES_BUFFER);  //Store current joints
  memset(this->currJoint, 0, NUM_BYTES_BUFFER);
  //Rotate Joint5 90 degree
  currJoint[4] = 90;  
  //Moving using kinematics
  goStrightLine(tmp, this->currJoint, 0.25e-4, 0.75e-10, 0.0, 0.0);
  setCurPos(0, 0, 0, 0, 90, 0);
  memcpy(this->buffer, this->currJoint, NUM_BYTES_BUFFER);
}

void ArmMoving::goFoldFromManual(){
  this->goHomeFromManual();
  // come back from home position to fold position
  // joint #5
  singleJointMove(DIR5_PIN, LOW, PUL5_PIN, (int)(180 / dl5));
  // joint #3
  singleJointMove(DIR3_PIN, HIGH, PUL3_PIN, 6569);
  // joint #2
  singleJointMove(DIR2_PIN, LOW, PUL2_PIN, 5582);

  // disable all joints
  digitalWrite(EN321_PIN, HIGH);
  digitalWrite(EN4_PIN, HIGH);
  digitalWrite(EN5_PIN, HIGH);
  digitalWrite(EN6_PIN, HIGH);
}

void ArmMoving::singleJointMove(uint8_t DIR_PIN, uint8_t DIR, uint8_t PUL_PIN, int totSteps, int delValue = 4000, int incValue = 7, int accRate = 530)
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

void ArmMoving::manualControl(char* DATA, float vel0, float acc0, float velini, float velfin){
  float tmp[6];
  memcpy(tmp, this->currJoint, NUM_BYTES_BUFFER); //Store current joints
  for (int i = 0; i < 6; ++i) {
    int steps = ANGLE_PER_COMMAND / DL[i];
    if (DATA[i] == POSITIVE_DIRECTION) {
      //Rotate positive direction
      this->currJoint[i] += ANGLE_PER_COMMAND;
      this->buffer[i] = this->currJoint[i] + 1;
    } 
    else if (DATA[i] == NEGATIVE_DIRECTION) {
      //Rotate negative direction
      this->currJoint[i] -= ANGLE_PER_COMMAND;
      this->buffer[i] = this->currJoint[i] - 1;
    }
  }
  goStrightLine(tmp, this->currJoint, vel0, acc0, velini, velfin);
}

void ArmMoving::listen(){
  this->listener.read();
  this->listener.validate();
  this->cmd = this->listener.getCommand();
}

void ArmMoving::move(){
  switch (this->cmd){
    case WAKEUP:
      this->wakeUp();
      Serial.println("ACK");
      break;
    case GO_HOME:
      this->goHomeFromManual();
      Serial.println("ACK");
      break;
    case GO_FOLD:
      this->goFoldFromManual();
      Serial.println("ACK");
      break;
    case MANUAL_MOVE:
      this->manualControl(this->listener.getData(), 0.25e-4, 0.1 * 0.75e-10, start_vel, end_vel);
      Serial.println("ACK");
      break;
    case STOP:
      goStrightLine(this->currJoint, this->buffer, 0.25e-4, 1.5*0.75e-10, 0.75 * velG, 0.25 * velG);
      memcpy(this->currJoint, this->buffer, NUM_BYTES_BUFFER);
      Serial.println("KCA");
      break;
    default:
      break; 
  }
}